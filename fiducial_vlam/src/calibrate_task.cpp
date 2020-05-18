
#include "calibrate.hpp"

#include "calibrate_classes.hpp"
#include "calibration_board_config.hpp"
#include "camera_calibration_parsers/parse.h"
#include "cv_utils.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_shared/string_printf.hpp"

#include <fstream>
#include <map>

namespace fiducial_vlam
{

// ==============================================================================
// GenerateCombinations class
// ==============================================================================

  template<typename Type>
  class GenerateCombinations
  {
    Type set_size_;
    Type selection_size_;
    std::vector<Type> selection_{};

  public:
    GenerateCombinations(Type set_size, Type selection_size) :
      set_size_{set_size}, selection_size_{std::min(selection_size, set_size)}
    {}

    const auto &selection() const
    { return selection_; }

    bool next()
    {
      Type i = 0;

      // Create the first sequence
      if (selection_.empty()) {
        while (i < selection_size_) {
          selection_.emplace_back(i + set_size_ - selection_size_);
          i += 1;
        }
        return true;
      }

      // Try to advance the sequence.
      while (i < selection_size_ && selection_[i] == i) {
        i += 1;
      }

      // Check for sequence done
      if (i == selection_size_) {
        return false;
      }

      // Adjust this entry
      selection_[i] -= 1;

      // and the previous entries
      while (i != 0) {
        i -= 1;
        selection_[i] = selection_[i + 1] - 1;
      }

      return true;
    }
  };

// ==============================================================================
// CalibrateCameraResult class
// ==============================================================================

  struct CalibrationStyles
  {
#define CALIBRATION_STYLES \
   CS(minimum_freedom) \
   CS(k1_free) \
   CS(k2_free) \
   CS(principal_point_free) \
   CS(k3_free) \
   CS(unequal_focal_lengths) \
   CS(tangent_distortion) \
   CS(custom) \
   CS(a_k1_free_b_fix_principal_point_free) \
   /* end of list */

    enum
    {
#define CS(n) n,
      unknown = -1,
      CALIBRATION_STYLES
      number_of_styles,
      range_twice_beg = a_k1_free_b_fix_principal_point_free,
      range_twice_end = number_of_styles,
    };

    static std::string name(int style)
    {
#undef CS
#define CS(n) if (style == n) return std::string(#n);
      CALIBRATION_STYLES
      return std::string("unknown style");
    }
  };

  struct CalibrateCameraResult
  {
    using ValueType = double;
    using JunctionIdIndexMap = std::map<JunctionId, std::size_t>;
    using CameraMatrixType = cv::Matx33d;
    using DistCoeffsType = cv::Vec<ValueType, 5>;
    using CameraVectorType = cv::Vec<ValueType, 4>;

    struct CalibrationResult
    {
      int calibration_style_{CalibrationStyles::unknown};
      std::vector<std::size_t> images_for_calibration_{};
      int flags_{0};
      ValueType reproject_error_{0.};
      ValueType internal_reproject_error_{0.};
      ValueType complete_reproject_error_{0.};
      CameraMatrixType camera_matrix_{};
      DistCoeffsType dist_coeffs_{};
      cv::Mat rvecs_{};
      cv::Mat tvecs_{};
      cv::Mat stdDeviationsIntrinsics_{}; // These cannot be Matx in opencv 4.1
      cv::Mat stdDeviationsExtrinsics_{};
      cv::Mat perViewErrors_{};
    };

    bool valid_{false};

    std::vector<cv::Mat> captured_images_marked_{};
    std::vector<std::vector<cv::Vec3f>> junctions_f_board_{};
    std::vector<std::vector<cv::Vec2f>> junctions_f_image_{};
    std::vector<JunctionIdIndexMap> junction_id_index_maps_{};

    rclcpp::Time calibration_time_{0, 0};

    std::vector<CalibrationResult> calibration_results_{};
  };

// ==============================================================================
// CalcStats class
// ==============================================================================

  template<typename TValue, int Tn>
  class CalcStats
  {
  public:
    using ValueType = TValue;
    using VectorType = cv::Vec<TValue, Tn>;

  private:
    int count_{0};
    VectorType mean_curr_{};
    VectorType mean_prev_{};
    VectorType s_curr_{};
    VectorType s_prev_{};

  public:
    void append(const VectorType &v)
    {
      count_++;

      if (count_ == 1) {
        // Set the very first values.
        mean_curr_ = v;
        s_curr_ = 0.;
      } else {
        // Save the previous values.
        mean_prev_ = mean_curr_;
        s_prev_ = s_curr_;

        // Update the current values.
        mean_curr_ = mean_prev_ + (v - mean_prev_) / count_;
        s_curr_ = s_prev_ + (v - mean_prev_).mul(v - mean_curr_);
      }
    }

    VectorType mean()
    {
      return mean_curr_;
    }

    VectorType std_dev()
    {
      VectorType std_dev{};
      cv::sqrt(s_curr_ / std::max(1, (count_ - 1)), std_dev);
      return std_dev;
    }
  };

// ==============================================================================
// CalibrateCameraReport class
// ==============================================================================

  class CalibrateCameraReport
  {
    const CalibrateContext &cal_cxt_;
    const CharucoboardConfig &cbm_;
    const rclcpp::Time &now_;
    std::unique_ptr<const CapturedImages> &captured_images_;
    uint32_t report_flags_;

    enum
    {
      REPORT_JUNCTION_ERRORS = 1U,
      REPORT_IMAGE_ERRORS = 2U,
      REPORT_CAL_ALL_IMAGES = 4U,
      REPORT_CAL_SUBSET_IMAGES = 8U,
      REPORT_CAL_SUBSET_STATS = 16U
    };

  public:
    CalibrateCameraReport(const CalibrateContext &cal_cxt,
                          const CharucoboardConfig &cbm,
                          const rclcpp::Time &now,
                          std::unique_ptr<const CapturedImages> &captured_images) :
      cal_cxt_{cal_cxt}, cbm_{cbm}, now_{now},
      captured_images_{captured_images},
      report_flags_{REPORT_CAL_ALL_IMAGES | REPORT_CAL_SUBSET_STATS}
    {}

    std::string to_date_string(const rclcpp::Time time)
    {
      auto nano = time.nanoseconds();
      auto secs = nano / 1000000000L;
      auto milli = (nano - secs * 1000000000L) / 1000000L;
      char time_buf[64];
      std::time_t t = secs;
      if (0 == strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S.", localtime(&t))) {
        time_buf[0] = 0;
      }
      std::string s(time_buf);
      auto milli_str{std::to_string(milli)};
      s.append(3 - milli_str.size(), '0').append(milli_str);
      return s;
    }

    std::string report_junction_errors(const CalibrateCameraResult &res,
                                       const CalibrateCameraResult::CalibrationResult &cal,
                                       std::size_t image_index)
    {
      std::string s{};
      std::vector<cv::Vec2f> reproject_image_points{};
      int bad_reprojection_count{0};
      int junction_count{0};
      double total_error_squared{0.0};

      // Project the object points onto the image so we can calculate the individual junction
      // reprojection errors.
      cv::projectPoints(res.junctions_f_board_[image_index],
                        cal.rvecs_.at<cv::Vec3d>(image_index, 0), cal.tvecs_.at<cv::Vec3d>(image_index, 0),
                        cal.camera_matrix_, cal.dist_coeffs_,
                        reproject_image_points);

      auto &junction_id_index_map_ = res.junction_id_index_maps_[image_index];
      auto &junctions_f_image = res.junctions_f_image_[image_index];
      for (JunctionId junction_id = 0; junction_id < cbm_.max_junction_id_; junction_id += 1) {
        auto p = junction_id_index_map_.find(junction_id);
        if (p == junction_id_index_map_.end()) {
          s.append("0.000 ");
        } else {
          auto index = p->second;
          auto error = cv::norm(reproject_image_points[index] - junctions_f_image[index]);
          total_error_squared += error * error;
          junction_count += 1;
          s.append(ros2_shared::string_print::f("%5.3f ", error));
          if (error > 1.) {
            bad_reprojection_count += 1;
          }
        }
        if (junction_id % cbm_.squares_x_m_1_ == cbm_.squares_x_m_1_ - 1) {
          s.append("\n");
        }
      }

      s.append(ros2_shared::string_print::f("Recalculated reprojection error: %5.3f (rms pixels)\n",
                                            std::sqrt(total_error_squared / junction_count)));

      if (bad_reprojection_count > 0) {
        s.append(ros2_shared::string_print::f("****** %d bad junction re-projection errors\n",
                                              bad_reprojection_count));
      }

      return s;
    }

    std::string camera_matrix_report(const CalibrateCameraResult::CameraMatrixType &camera_matrix,
                                     const CalibrateCameraResult::DistCoeffsType &dist_coeffs,
                                     const cv::Mat &stdDeviationsIntrinsics)
    {
      std::string s;

      s.append(ros2_shared::string_print::f("fx, fy, cx, cy: %f %f %f %f\n",
                                            camera_matrix(0, 0),
                                            camera_matrix(1, 1),
                                            camera_matrix(0, 2),
                                            camera_matrix(1, 2)));
      s.append(ros2_shared::string_print::f("std dev fx, fy, cx, cy: %f %f %f %f\n",
                                            stdDeviationsIntrinsics.at<double>(0),
                                            stdDeviationsIntrinsics.at<double>(1),
                                            stdDeviationsIntrinsics.at<double>(2),
                                            stdDeviationsIntrinsics.at<double>(3)));

      s.append(ros2_shared::string_print::f("k1, k2, p1, p2, k3: %f %f %f %f %f\n",
                                            dist_coeffs(0),
                                            dist_coeffs(1),
                                            dist_coeffs(2),
                                            dist_coeffs(3),
                                            dist_coeffs(4)));
      s.append(ros2_shared::string_print::f("std dev k1, k2, p1, p2, k3: %f %f %f %f %f\n",
                                            stdDeviationsIntrinsics.at<double>(4),
                                            stdDeviationsIntrinsics.at<double>(5),
                                            stdDeviationsIntrinsics.at<double>(6),
                                            stdDeviationsIntrinsics.at<double>(7),
                                            stdDeviationsIntrinsics.at<double>(8)));

      return s;
    }

    std::string create_one_calibration_report(const CalibrateCameraResult &res,
                                              const CalibrateCameraResult::CalibrationResult &cal)
    {
      std::string s{};
      s.append(ros2_shared::string_print::f("\nCamera calibration style %d, (%s)\n",
                                            cal.calibration_style_,
                                            CalibrationStyles::name(cal.calibration_style_).c_str()));

      if (cal.images_for_calibration_.empty()) {
        s.append("Using all captured images\n");
      } else {
        s.append("Using  captured images: ");
        for (auto ifc : cal.images_for_calibration_) {
          if (ifc != cal.images_for_calibration_[0]) {
            s.append(", ");
          }
          s.append(ros2_shared::string_print::f("%d", ifc));
        }
        s.append("\n");
      }

      s.append(camera_matrix_report(cal.camera_matrix_,
                                    cal.dist_coeffs_,
                                    cal.stdDeviationsIntrinsics_));

      for (size_t i = 0; i < cal.perViewErrors_.rows; i += 1) {
        auto image_index = cal.images_for_calibration_.empty() ? i : cal.images_for_calibration_[i];
        if (report_flags_ & REPORT_IMAGE_ERRORS) {
          s.append(ros2_shared::string_print::f(
            "Image %d, %s - Reprojection error %6.4f\n",
            image_index,
            to_date_string(captured_images_->captured_images()[image_index]->time_stamp()).c_str(),
            cal.perViewErrors_.at<double>(i, 0)));
        }
      }

      s.append(ros2_shared::string_print::f("Total reprojection error %6.4f, internal %6.4f",
                                            cal.reproject_error_, cal.internal_reproject_error_));

      if (!cal.images_for_calibration_.empty()) {
        s.append(ros2_shared::string_print::f(", complete %6.4f",
                                              cal.complete_reproject_error_));
      }

      s.append("\n");
      return s;
    }

    std::string create_averaged_calibration_report(const CalibrateCameraResult &res)
    {
      std::string s;

      // Calculate the mean and std dev of all cals that use a subset of the calibrated images.
      CalibrateCameraResult::CalibrationResult average_cal;

      CalcStats<CalibrateCameraResult::ValueType, 4> cv_stats{};
      CalcStats<CalibrateCameraResult::ValueType, 5> dc_stats{};
      CalcStats<CalibrateCameraResult::ValueType, 1> re_stats{};

      int calibration_style{CalibrationStyles::unknown};
      int flags{0};
      uint32_t set_size{0};
      uint32_t selection_size{0};
      uint32_t combinations_size{0};


      for (auto &cal : res.calibration_results_) {
        if (!cal.images_for_calibration_.empty()) {
          combinations_size += 1;

          if (calibration_style == CalibrationStyles::unknown) {
            calibration_style = cal.calibration_style_;
            flags = cal.flags_;
            selection_size = cal.images_for_calibration_.size();
            set_size = res.junctions_f_board_.size();
          }

          CalibrateCameraResult::CameraVectorType cal_cv{cal.camera_matrix_(0, 0),
                                                         cal.camera_matrix_(1, 1),
                                                         cal.camera_matrix_(0, 2),
                                                         cal.camera_matrix_(1, 2)};


          cv_stats.append(cal_cv);
          dc_stats.append(cal.dist_coeffs_);
          re_stats.append(cal.complete_reproject_error_);
        }
      }

      auto cv_mean = cv_stats.mean();
      auto dc_mean = dc_stats.mean();
      auto re_mean = re_stats.mean();
      auto cv_std_dev = cv_stats.std_dev();
      auto dc_std_dev = dc_stats.std_dev();
      auto re_std_dev = re_stats.std_dev();

      CalibrateCameraResult::CameraMatrixType camera_matrix{};
      camera_matrix(0, 0) = cv_mean(0);
      camera_matrix(1, 1) = cv_mean(1);
      camera_matrix(0, 2) = cv_mean(2);
      camera_matrix(1, 2) = cv_mean(3);
      camera_matrix(2, 2) = 1.;

      auto stdDeviationsIntrinsics = cv::Mat(9, 1, CV_64F);
      stdDeviationsIntrinsics.at<double>(0) = cv_std_dev(0);
      stdDeviationsIntrinsics.at<double>(1) = cv_std_dev(1);
      stdDeviationsIntrinsics.at<double>(2) = cv_std_dev(2);
      stdDeviationsIntrinsics.at<double>(3) = cv_std_dev(3);
      stdDeviationsIntrinsics.at<double>(4) = dc_std_dev(0);
      stdDeviationsIntrinsics.at<double>(5) = dc_std_dev(1);
      stdDeviationsIntrinsics.at<double>(6) = dc_std_dev(2);
      stdDeviationsIntrinsics.at<double>(7) = dc_std_dev(3);
      stdDeviationsIntrinsics.at<double>(8) = dc_std_dev(4);

      s.append(ros2_shared::string_print::f("\nSummary of %d combinations of %d out of %d captured images. calibration style %d, (%s)\n",
                                            combinations_size, selection_size, set_size,
                                            calibration_style,
                                            CalibrationStyles::name(calibration_style).c_str()));
      s.append(camera_matrix_report(camera_matrix, dc_mean, stdDeviationsIntrinsics));
      s.append(ros2_shared::string_print::f("Average complete reprojection error %6.4f, std dev %6.4f",
                                            re_mean(0), re_std_dev(0)));

      return s;
    }

    std::string create(const CalibrateCameraResult &res,
                       const CalibrateCameraResult::CalibrationResult &cal)
    {
      std::string s{};
      s.append(ros2_shared::string_print::f("Camera calibration done on %s.\nWith %dx%d images.\n",
                                            to_date_string(now_).c_str(),
                                            captured_images_->image_size().width,
                                            captured_images_->image_size().height));

      for (auto &one_cal : res.calibration_results_) {
        if (((report_flags_ & REPORT_CAL_ALL_IMAGES) && one_cal.images_for_calibration_.empty()) ||
            ((report_flags_ & REPORT_CAL_SUBSET_IMAGES) && !one_cal.images_for_calibration_.empty())) {
          s.append(create_one_calibration_report(res, one_cal));
        }
      }

      if (report_flags_ & REPORT_CAL_SUBSET_STATS) {
        s.append(create_averaged_calibration_report(res));
      }

      if (report_flags_ & REPORT_JUNCTION_ERRORS) {
        s.append(ros2_shared::string_print::f(
          "\nIndividual junction re-projection errors for calibration style %d (%s).\n",
          cal.calibration_style_,
          CalibrationStyles::name(cal.calibration_style_).c_str()));

        for (size_t i = 0; i < cal.perViewErrors_.rows; i += 1) {
          s.append(ros2_shared::string_print::f(
            "Image %d, %s - Reprojection error %5.3f\n",
            i, to_date_string(captured_images_->captured_images()[i]->time_stamp()).c_str(),
            cal.perViewErrors_.at<double>(i, 0)));
          s.append(report_junction_errors(res, cal, i));
          s.append("\n");
        }
      }

      return s;
    }
  };

// ==============================================================================
// CalibrateCameraTaskImpl class
// ==============================================================================

  class CalibrateCameraTaskImpl : public CalibrateCameraTaskInterface
  {
    using MarkersHomography = std::map<ArucoId, std::tuple<cv::Mat, std::size_t>>;

    const CalibrateContext cal_cxt_; // copy on construction so it doesn't change
    const CharucoboardConfig cbm_;
    const rclcpp::Time now_;
    std::unique_ptr<const CapturedImages> captured_images_;

  public:
    CalibrateCameraTaskImpl(const CalibrateContext &cal_cxt,
                            const rclcpp::Time &now,
                            std::unique_ptr<const CapturedImages> captured_images) :
      cal_cxt_{cal_cxt},
      cbm_(cal_cxt.cal_squares_x_, cal_cxt.cal_squares_y_, cal_cxt.cal_square_length_,
           cal_cxt.cal_upper_left_white_not_black_, cal_cxt.cal_marker_length_),
      now_{now},
      captured_images_{std::move(captured_images)}
    {}

    CalibrateCameraTaskResult calculate_calibration() override
    {
      CalibrateCameraResult res;

      // Do some per captured image tasks to prepare for calibration.
      prepare_captured_images(res);

      // Loop over the images finding the checkerboard junctions
      for (auto &captured_iamge : captured_images_->captured_images()) {
        interpolate_junction_locations(captured_iamge, res);
      }

      // Loop over all the possible calibration styles.
      for (int calib_style = 0; calib_style < CalibrationStyles::number_of_styles; calib_style += 1) {
        do_calibration(calib_style, std::vector<size_t>{}, res);
      }

      // find the desired calibration style.
      auto calibration_style = std::max(0, std::min(CalibrationStyles::number_of_styles - 1,
                                                    cal_cxt_.cal_calibration_style_to_save_));

      // Try all combinations of captured images.
      if (cal_cxt_.cal_bootstrap_reserve_fraction_ > 0.0 &&
          cal_cxt_.cal_bootstrap_reserve_fraction_ < 1.0) {
        calibrate_with_image_sets(calibration_style, res);
      }

      // Save the calibration file.
      auto &cal = res.calibration_results_[calibration_style];
      auto s{save_calibration(now_, cal)};

      // Save the captured images
      s.append(save_captured_and_marked_images(res));

      // Create and save the report
      s.append(CalibrateCameraReport(cal_cxt_, cbm_, now_, captured_images_).create(res, cal));
      save_report(s);

      // Return the report and the marked images.
      CalibrateCameraTaskResult task_res{};
      task_res.calibration_report_ = std::move(s);
      task_res.captured_images_marked_ = std::move(res.captured_images_marked_);

      return task_res;
    }

    void calibrate_with_image_sets(int calibration_style,
                                   CalibrateCameraResult &res)
    {
      std::size_t set_size = res.junctions_f_image_.size();
      std::size_t reserve_size = static_cast<size_t>(std::round(set_size * cal_cxt_.cal_bootstrap_reserve_fraction_));
      std::size_t selection_size = std::max(4UL, std::min(set_size, set_size - reserve_size));

      GenerateCombinations<std::size_t> combinations(set_size, selection_size);
      while (combinations.next()) {
        do_calibration(calibration_style, combinations.selection(), res);
      }
    }

    void do_calibration(int calibration_style,
                        const std::vector<size_t> &images_for_calibration,
                        CalibrateCameraResult &res)
    {
      CalibrateCameraResult::CalibrationResult cal{};

      cal.calibration_style_ = calibration_style;
      cal.images_for_calibration_ = images_for_calibration;

      // Pick out the images to use for calibration. Here we copy points
      // from boards of interest to a separate vector.
      std::vector<std::vector<cv::Vec3f>> junctions_f_board_tmp{};
      std::vector<std::vector<cv::Vec2f>> junctions_f_image_tmp{};
      if (!images_for_calibration.empty()) {
        for (auto ifc : images_for_calibration) {
          junctions_f_board_tmp.insert(junctions_f_board_tmp.end(), res.junctions_f_board_[ifc]);
          junctions_f_image_tmp.insert(junctions_f_image_tmp.end(), res.junctions_f_image_[ifc]);
        }
      }

      // Create references to the vectors to use for calibration.
      std::vector<std::vector<cv::Vec3f>> &junctions_f_board{!images_for_calibration.empty() ?
                                                             junctions_f_board_tmp :
                                                             res.junctions_f_board_};
      std::vector<std::vector<cv::Vec2f>> &junctions_f_image{!images_for_calibration.empty() ?
                                                             junctions_f_image_tmp :
                                                             res.junctions_f_image_};

      auto image_size{captured_images_->image_size()};

      // For these styles, do two calibrations.
      if (calibration_style >= CalibrationStyles::range_twice_beg &&
          calibration_style < CalibrationStyles::range_twice_end) {
        switch (calibration_style) {
          default:
          case CalibrationStyles::a_k1_free_b_fix_principal_point_free:
            cal.flags_ = cv::CALIB_FIX_PRINCIPAL_POINT |
                         cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
                         cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
            cal.camera_matrix_(0, 0) = 1.0;
            cal.camera_matrix_(1, 1) = 1.0;
            cal.camera_matrix_(0, 2) = image_size.width / 2;
            cal.camera_matrix_(1, 2) = image_size.height / 2;
            cal.camera_matrix_(2, 2) = 1.0;
            break;
        }

        cal.reproject_error_ = calibrateCamera(
          junctions_f_board, junctions_f_image,
          image_size,
          cal.camera_matrix_, cal.dist_coeffs_,
          cv::noArray(), cv::noArray(),
          cv::noArray(), cv::noArray(), cv::noArray(),
          cal.flags_);
      }

      // Set up the flags and calibration values for each style of calibration
      switch (calibration_style) {
        default:
        case CalibrationStyles::minimum_freedom:
          cal.flags_ = cv::CALIB_FIX_PRINCIPAL_POINT |
                       cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
                       cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
          cal.camera_matrix_(0, 0) = 1.0;
          cal.camera_matrix_(1, 1) = 1.0;
          cal.camera_matrix_(0, 2) = image_size.width / 2;
          cal.camera_matrix_(1, 2) = image_size.height / 2;
          cal.camera_matrix_(2, 2) = 1.0;
          break;
        case CalibrationStyles::k1_free:
          cal.flags_ = cv::CALIB_FIX_PRINCIPAL_POINT |
                       cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
                       cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
          cal.camera_matrix_(0, 0) = 1.0;
          cal.camera_matrix_(1, 1) = 1.0;
          cal.camera_matrix_(0, 2) = image_size.width / 2;
          cal.camera_matrix_(1, 2) = image_size.height / 2;
          cal.camera_matrix_(2, 2) = 1.0;
          break;
        case CalibrationStyles::k2_free:
          cal.flags_ = cv::CALIB_FIX_PRINCIPAL_POINT |
                       cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
                       cv::CALIB_FIX_K3;
          cal.camera_matrix_(0, 0) = 1.0;
          cal.camera_matrix_(1, 1) = 1.0;
          cal.camera_matrix_(0, 2) = image_size.width / 2;
          cal.camera_matrix_(1, 2) = image_size.height / 2;
          cal.camera_matrix_(2, 2) = 1.0;
          break;
        case CalibrationStyles::principal_point_free:
          cal.flags_ = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
                       cv::CALIB_FIX_K3;
          cal.camera_matrix_(0, 0) = 1.0;
          cal.camera_matrix_(1, 1) = 1.0;
          break;
        case CalibrationStyles::k3_free:
          cal.flags_ = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST;
          cal.camera_matrix_(0, 0) = 1.0;
          cal.camera_matrix_(1, 1) = 1.0;
          break;
        case CalibrationStyles::unequal_focal_lengths:
          cal.flags_ = cv::CALIB_ZERO_TANGENT_DIST;
          break;
        case CalibrationStyles::tangent_distortion:
          cal.flags_ = 0;
          break;
        case CalibrationStyles::custom:
          cal.flags_ = cv::CALIB_USE_INTRINSIC_GUESS |
                       cv::CALIB_FIX_PRINCIPAL_POINT |
                       cv::CALIB_FIX_FOCAL_LENGTH | cv::CALIB_FIX_ASPECT_RATIO |
                       cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
#if 1
          cal.camera_matrix_(0, 0) = 964.38450275274;
          cal.camera_matrix_(1, 1) = 964.384502752749;
          cal.camera_matrix_(0, 2) = 656.0148256658157;
          cal.camera_matrix_(1, 2) = 364.7037206674901;
          cal.camera_matrix_(2, 2) = 1.0;
          cal.dist_coeffs_(0) = -0.00273078850339132;
          cal.dist_coeffs_(1) = 0.06090272742385654;
          cal.dist_coeffs_(2) = 0.;
          cal.dist_coeffs_(3) = 0.;
          cal.dist_coeffs_(4) = -0.09995825912626571;
#else
          cal.camera_matrix_(0, 0) = 699.3550;
          cal.camera_matrix_(1, 1) = 699.3550;
          cal.camera_matrix_(0, 2) = 650.0850;
          cal.camera_matrix_(1, 2) = 354.6600;
          cal.camera_matrix_(2, 2) = 1.0;
          cal.dist_coeffs_(0) = -0.1716;
          cal.dist_coeffs_(1) = 0.0246;

          cal.camera_matrix_(0, 0) = 700.9050;
          cal.camera_matrix_(1, 1) = 700.9050;
          cal.camera_matrix_(0, 2) = 655.5400;
          cal.camera_matrix_(1, 2) = 358.5940;
          cal.camera_matrix_(2, 2) = 1.0;
          cal.dist_coeffs_(0) = -0.1681;
          cal.dist_coeffs_(1) = 0.0205;
#endif
          break;
        case CalibrationStyles::a_k1_free_b_fix_principal_point_free:
          cal.flags_ = cv::CALIB_USE_INTRINSIC_GUESS |
                       cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
                       cv::CALIB_FIX_K3;
          break;
      }

      // Do the calibration.
      cal.reproject_error_ = calibrateCamera(
        junctions_f_board, junctions_f_image,
        image_size,
        cal.camera_matrix_, cal.dist_coeffs_,
        cal.rvecs_, cal.tvecs_,
        cal.stdDeviationsIntrinsics_, cal.stdDeviationsExtrinsics_, cal.perViewErrors_,
        cal.flags_);

      // Do our own calculation of the re-projection error.
      calculate_reprojection_errors(junctions_f_board, junctions_f_image, cal);

      res.calibration_results_.emplace_back(std::move(cal));
    }

    void interpolate_junction_locations(std::shared_ptr<ImageHolder> captured_image,
                                        CalibrateCameraResult &res)
    {
      // Create a bunch of vectors to store the data we find.
      std::vector<cv::Vec3f> js_f_board{};
      std::vector<cv::Vec2f> js_f_image{};
      CalibrateCameraResult::JunctionIdIndexMap j_id_index_map{};

      // Figure out which image we can mark.
      auto &captured_image_marked = res.captured_images_marked_[res.junctions_f_board_.size()];

      // Calculate the local homography for each found marker and build a map indexed by
      // the ArucoId.
      auto markers_homography = calculate_markers_homography(captured_image);

      // Walk over all the junctions on the board.
      for (JunctionId junction_id = 0; junction_id < cbm_.max_junction_id_; junction_id += 1) {

        // Get the two adjacent aruco ids.
        auto adjacent_aruco_ids = cbm_.get_adjacent_arucos(junction_id);
        auto adjacent_aruco_closest_corner_idx = cbm_.get_adjacent_arucos_closest_corner(junction_id);

        // Figure out where this junction is on the facade.
        auto junction_location = cbm_.junction_id_to_junction_location(junction_id);
        std::vector<cv::Point2f> junction_f_facade{cv::Point2f(junction_location(0), junction_location(1))};

        // For both of the adjacent aruco markers, check that they have been detected, and
        // use the local marker homography to figure out where the junction should be in the
        // image.
        std::vector<cv::Point2f> local_junctions_f_image{};
        std::vector<cv::Point2f> closest_corners_f_image{};
        for (std::size_t i = 0; i < adjacent_aruco_ids.size(); i += 1) {

          // Find the local homography for this marker
          auto find_ptr = markers_homography.find(adjacent_aruco_ids[i]);
          if (find_ptr != markers_homography.end()) {

            // Map the junction location on the board to a the junction location in the image using the
            // homography transformation of the adjacent aruco marker.
            std::vector<cv::Point2f> junction_f_image;
            cv::perspectiveTransform(junction_f_facade, junction_f_image, std::get<0>(find_ptr->second));
            local_junctions_f_image.emplace_back(junction_f_image[0]);

            // Pick out the location of the corner of this marker that is closest to the
            // junction.
            auto &aruco_corners = captured_image->aruco_corners()[std::get<1>(find_ptr->second)];
            closest_corners_f_image.emplace_back(aruco_corners[adjacent_aruco_closest_corner_idx[i]]);
          }
        }

        // If neither of the markers was found, then continue to the next
        // junction
        if (local_junctions_f_image.size() < 1) {
          continue;
        }

        // Average the junction image location if both of the markers have been detected.
        if (local_junctions_f_image.size() > 1) {
          local_junctions_f_image[0] = (local_junctions_f_image[0] + local_junctions_f_image[1]) / 2.0;
        }

        // We want to figure a custom window size for doing the sub-pixel corner refinement.
        // This is done by using a window size that is smaller than the distance from the
        // junction to the closest aruco corner.
        auto win_size = calculate_sub_pix_win_size(local_junctions_f_image[0], closest_corners_f_image);

        // Find the junction image location with sub pixel accuracy.
        local_junctions_f_image.resize(1);
        cv::cornerSubPix(captured_image->gray(), local_junctions_f_image, win_size, cv::Size(),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                          30, DBL_EPSILON));

        // Add these junction locations (f_image, f_board) to the list.
        std::size_t index = js_f_board.size();
        js_f_board.emplace_back(cv::Vec3f(junction_location(0), junction_location(1), 0.));
        js_f_image.emplace_back(local_junctions_f_image[0]);
        j_id_index_map.emplace(junction_id, index);

        AnnotateImages::with_detected_junction(captured_image_marked, local_junctions_f_image[0], win_size);
      }

      res.junctions_f_board_.emplace_back(std::move(js_f_board));
      res.junctions_f_image_.emplace_back(std::move(js_f_image));
      res.junction_id_index_maps_.emplace_back(std::move(j_id_index_map));
    }

    MarkersHomography calculate_markers_homography(std::shared_ptr<ImageHolder> captured_image)
    {
      MarkersHomography markers_homography{};

      for (std::size_t idx = 0; idx < captured_image->aruco_ids().size(); idx += 1) {
        ArucoId aruco_id(captured_image->aruco_ids()[idx]);
        auto &aruco_corners_f_image(captured_image->aruco_corners()[idx]);

        std::vector<cv::Point2f> aruco_corners_f_facade{};
        auto const &acff(cbm_.to_aruco_corners_f_facade(aruco_id));
        for (std::size_t c = 0; c < 4; c += 1) {
          aruco_corners_f_facade.emplace_back(cv::Point2f(acff(0, c), acff(1, c)));
        }

        auto homo = findHomography(aruco_corners_f_facade, aruco_corners_f_image);

        markers_homography.emplace(aruco_id, std::tuple<cv::Mat, std::size_t>{homo, idx});
      }

      return markers_homography;
    }

    // Figure out how big to make the window that will be used for the sub-pixel refinement to find
    // the image location at the junction of two black squares on the configuration target.
    // As input to the routine, we have the image coordinates where we think the junction will
    // be and the image coordinates of the closest corners of the aruco markers. We want the window
    // size as large as possible be it can't include the aruco corner because the sub-pixel algorithm
    // might lock on to the aruco corner instead of the black square junction.
    cv::Size calculate_sub_pix_win_size(cv::Point2f &mean_junction_f_image,
                                        std::vector<cv::Point2f> &closest_corners_f_image)
    {
      // Figure out the window using one of the aruco corners.
      auto size2f{cv::Size2f(std::abs(mean_junction_f_image.x - closest_corners_f_image[0].x),
                             std::abs(mean_junction_f_image.y - closest_corners_f_image[0].y))};

      // If two aruco markers were found, then pick the smallest window.
      if (closest_corners_f_image.size() > 1) {
        auto size2f1{cv::Size2f(std::abs(mean_junction_f_image.x - closest_corners_f_image[1].x),
                                std::abs(mean_junction_f_image.y - closest_corners_f_image[1].y))};
        size2f = cv::Size2f(std::min(size2f.width, size2f1.width),
                            std::min(size2f.height, size2f1.height));
      }

      cv::Size win_size{static_cast<int>(std::floor(size2f.width)),
                        static_cast<int>(std::floor(size2f.height))};

      // do some sanity checks.
      win_size = cv::Size{win_size.width - 1, win_size.height - 1}; // remove 1 pixel for safety
      win_size = cv::Size{std::min(10, std::max(2, win_size.width)),  // min: 2, max: 10
                          std::min(10, std::max(2, win_size.height))};
      return win_size;
    }


    void calculate_reprojection_errors(const std::vector<std::vector<cv::Vec3f>> &junctions_f_board,
                                       const std::vector<std::vector<cv::Vec2f>> &junctions_f_image,
                                       CalibrateCameraResult::CalibrationResult &cal)
    {
      std::vector<bool> included(cal.images_for_calibration_.size(), false);
      for (auto image_index : cal.images_for_calibration_) {
        included[image_index] = true;
      }

      decltype(cal.reproject_error_) internal_error_acc(0.);
      decltype(cal.reproject_error_) complete_error_acc(0.);
      uint32_t internal_count{0};
      uint32_t complete_count{0};
      for (size_t image_index = 0; image_index < junctions_f_board.size(); image_index += 1) {
        auto error = calc_single_image_reprojection_error(junctions_f_board[image_index],
                                                          junctions_f_image[image_index],
                                                          cal);

        auto error_sq = error * error;
        complete_error_acc += error_sq;
        complete_count += 1;
        if (cal.images_for_calibration_.empty() || included[image_index]) {
          internal_error_acc += error_sq;
          internal_count += 1;
        }
      }

      cal.internal_reproject_error_ = std::sqrt(internal_error_acc / internal_count);
      cal.complete_reproject_error_ = std::sqrt(complete_error_acc / complete_count);
    }

    double calc_single_image_reprojection_error(const std::vector<cv::Vec3f> &junctions_f_board,
                                                const std::vector<cv::Vec2f> &junctions_f_image,
                                                const CalibrateCameraResult::CalibrationResult &cal)
    {
      // Figure out the location of the board relative to the camera.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(junctions_f_board, junctions_f_image,
                   cal.camera_matrix_, cal.dist_coeffs_,
                   rvec, tvec);

      // Project the object points onto the image so we can calculate the individual junction
      // reprojection errors.
      std::vector<cv::Vec2f> junctions_f_image_reproject{};
      cv::projectPoints(junctions_f_board,
                        rvec, tvec,
                        cal.camera_matrix_, cal.dist_coeffs_,
                        junctions_f_image_reproject);

      double total_error_squared{0.0};
      for (size_t i = 0; i < junctions_f_image.size(); i += 1) {
        auto error = cv::norm(junctions_f_image_reproject[i] - junctions_f_image[i]);
        total_error_squared += error * error;
      }

      return std::sqrt(total_error_squared / junctions_f_image.size());
    }

    void prepare_captured_images(CalibrateCameraResult &res)
    {
      for (auto &ci : captured_images_->captured_images()) {
        // Redetect the aruco corners using precision
        ci->detect_markers(cbm_, true);

        // Create the color marked images for annotating
        cv::Mat cim{};
        cv::cvtColor(ci->gray(), cim, cv::COLOR_GRAY2BGR);
        res.captured_images_marked_.push_back(cim);

        // Annotate the charuco markers.
        AnnotateImages::with_detected_markers(cim, ci->aruco_corners(), ci->aruco_ids());
      }
    }

    std::string save_calibration(const rclcpp::Time &now,
                                 CalibrateCameraResult::CalibrationResult &cal)
    {
      // Build up a camera_info message with the calibration data.
      sensor_msgs::msg::CameraInfo camera_info;
      camera_info.header.stamp = now;
      camera_info.width = captured_images_->image_size().width;
      camera_info.height = captured_images_->image_size().height;
      camera_info.distortion_model = "plumb_bob";

      auto &cm = cal.camera_matrix_;
      camera_info.k = std::array<double, 9>{cm(0, 0), cm(0, 1), cm(0, 2),
                                            cm(1, 0), cm(1, 1), cm(1, 2),
                                            cm(2, 0), cm(2, 1), cm(2, 2)};
      auto &dm = cal.dist_coeffs_;
      camera_info.d = std::vector<double>{dm(0), dm(1), dm(2), dm(3), dm(4)};

      camera_info.r = std::array<double, 9>{1, 0, 0,
                                            0, 1, 0,
                                            0, 0, 1};

      camera_info.p = std::array<double, 12>{cm(0, 0), cm(0, 1), cm(0, 2), 0.,
                                             cm(1, 0), cm(1, 1), cm(1, 2), 0.,
                                             cm(2, 0), cm(2, 1), cm(2, 2), 0.};

      auto filename{get_camera_info_file_name(cal_cxt_)};
      camera_calibration_parsers::writeCalibration(filename,
                                                   cal_cxt_.cal_camera_name_,
                                                   camera_info);

      return std::string{ros2_shared::string_print::f("Calibration for camera '%s' saved to file: %s\n",
                                                      cal_cxt_.cal_camera_name_.c_str(),
                                                      filename.c_str())};
    }

    std::string save_captured_and_marked_images(const CalibrateCameraResult &res)
    {
      cv::FileStorage fs_header(get_captured_image_file_name(cal_cxt_),
                                cv::FileStorage::WRITE);

      fs_header << "width" << captured_images_->image_size().width
                << "height" << captured_images_->image_size().height
                << "imageNames" << "[";

      auto captured_images = captured_images_->captured_images();
      for (int i = 0; i < captured_images.size(); i += 1) {

        auto image_file_name{get_captured_image_file_name(cal_cxt_, i)};
        auto marked_image_file_name{get_marked_image_file_name(cal_cxt_, i)};
        cv::imwrite(image_file_name, captured_images[i]->gray());
        cv::imwrite(marked_image_file_name, res.captured_images_marked_[i]);

        fs_header << "{:"
                  << "name" << image_file_name
                  << "stamp" << std::to_string(captured_images[i]->time_stamp().nanoseconds())
                  << "clock" << captured_images[i]->time_stamp().get_clock_type()
                  << "},";
      }

      fs_header << "]";
      fs_header.release();
      return std::string{};
    }

    void save_report(const std::string s)
    {
      std::ofstream file;
      file.open(get_calibration_report_file_name(cal_cxt_));
      file << s;
      file.close();
    }
  };


  std::unique_ptr<CalibrateCameraTaskInterface> make_calibrate_camera_task(
    const CalibrateContext &cal_cxt,
    const rclcpp::Time &now,
    std::unique_ptr<const CapturedImages> captured_images)
  {
    return std::make_unique<CalibrateCameraTaskImpl>(cal_cxt, now, std::move(captured_images));
  }
}
