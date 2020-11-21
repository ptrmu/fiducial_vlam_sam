
#include "fiducial_math.hpp"

#include <map>
#include "observation.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vloc_context.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// KalmanFilterX class
// ==============================================================================

  // A class the implements a Kalman Filter. This filter is implemented using
  // opencv's Matx matrices. This implementation follows the opencv KalmanFilter
  // implementation.

  template<typename Type, int StateDims, int MeasurementDims>
  class KalmanFilterX
  {
  public:
    using ValueType = Type;
    using StateType = cv::Vec<Type, StateDims>;
    using MeasurementType = cv::Vec<Type, MeasurementDims>;
    using StateCovType = cv::Matx<Type, StateDims, StateDims>;
    using MeasurementCovType = cv::Matx<Type, MeasurementDims, MeasurementDims>;

  private:
    // temporary matrices
    cv::Matx<Type, StateDims, StateDims> temp1;
    cv::Matx<Type, MeasurementDims, StateDims> temp2;
    cv::Matx<Type, MeasurementDims, MeasurementDims> temp3;
    cv::Matx<Type, MeasurementDims, StateDims> temp4;
    cv::Vec<Type, MeasurementDims> temp5;

    StateType x_k_prime;                              // predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    StateCovType error_cov_k_prime;                   // priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    cv::Matx<Type, StateDims, MeasurementDims> gain;  // Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)

  public:
    cv::Matx<Type, StateDims, StateDims> transition_matrix;         // state transition matrix (A)
    cv::Matx<Type, MeasurementDims, StateDims> measurement_matrix;  // measurement matrix (H)
    StateCovType process_noise_cov;                                 // process noise covariance matrix (Q)
    MeasurementCovType measurement_noise_cov;                       // measurement noise covariance matrix (R)

    struct FilterState
    {
      StateType x_k;                                  // corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
      StateCovType error_cov_k;                       // posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
    };

    const StateType &predict(FilterState &fs); //
    const StateType &update(FilterState &fs,
                            const MeasurementType &measurement); //
  };

  template<typename Type, int StateDims, int MeasureDims>
  const typename KalmanFilterX<Type, StateDims, MeasureDims>::StateType &
  KalmanFilterX<Type, StateDims, MeasureDims>::predict(FilterState &fs)
  {
    // update the state: x'(k) = A*x(k)
    x_k_prime = transition_matrix * fs.x_k;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transition_matrix * fs.error_cov_k;

    // P'(k) = temp1*At + Q
    cv::gemm(temp1, transition_matrix, 1, process_noise_cov, 1, error_cov_k_prime, cv::GEMM_2_T);

    // handle the case when there will be measurement before the next predict.
    fs.x_k = x_k_prime;
    fs.error_cov_k = error_cov_k_prime;

    return x_k_prime;
  }

  template<typename Type, int StateDims, int MeasureDims>
  const typename KalmanFilterX<Type, StateDims, MeasureDims>::StateType &
  KalmanFilterX<Type, StateDims, MeasureDims>::update(FilterState &d,
                                                      const MeasurementType &measurement)
  {
    // temp2 = H*P'(k)
    temp2 = measurement_matrix * error_cov_k_prime;

    // temp3 = temp2*Ht + R
    cv::gemm(temp2, measurement_matrix, 1, measurement_noise_cov, 1, temp3, cv::GEMM_2_T);

    // temp4 = inv(temp3)*temp2 = Kt(k)
    solve(temp3, temp2, temp4, cv::DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurement_matrix * x_k_prime;

    // x(k) = x'(k) + K(k)*temp5
    d.x_k = x_k_prime + gain * temp5;

    // P(k) = P'(k) - K(k)*temp2
    d.error_cov_k = error_cov_k_prime - gain * temp2;

    return d.x_k;
  }

// ==============================================================================
// CVFType class
// ==============================================================================

// Constant Velocity Kalman Filter. This class has a 2 element state vector. The
// state variables are position and velocity.

  class CVFType : public KalmanFilterX<float, 2, 1>
  {
  public:
    CVFType() = default; //
    void init(ValueType dt,
              ValueType process_std_dev,
              ValueType measurement_std_dev); //
    void init_filter_state(FilterState &fs,
                           ValueType measurement_0); //
  };

  void CVFType::init(ValueType dt,
                     ValueType process_std_dev,
                     ValueType measurement_std_dev)
  {
    transition_matrix = {1, dt, 0, 1};
    measurement_matrix = {1, 0};
    process_noise_cov = StateCovType{dt * dt, dt, dt, 1.} * process_std_dev * process_std_dev;
    measurement_noise_cov = MeasurementCovType{measurement_std_dev * measurement_std_dev};
  }

  void CVFType::init_filter_state(FilterState &fs,
                                  ValueType measurement_0)
  {
    fs.x_k = StateType{measurement_0, 0.}; // (phi, delta_phi)
    fs.error_cov_k = {10., 10., 10., 10.}; // Large uncertainty to start with.
  }

// ==============================================================================
// MarkerFilter class
// ==============================================================================

  class MarkerFilter
  {
    using FilterStates = std::array<CVFType::FilterState, 8>;

    bool observed_{false};
    int frames_skipped_{0};
    FilterStates filter_states_{};

    template<typename Func>
    static void for_each(FilterStates &filter_states,
                         std::vector<cv::Point2f> &marker_corners,
                         Func func)
    {
      func(filter_states[0], marker_corners[0].x);
      func(filter_states[1], marker_corners[0].y);
      func(filter_states[2], marker_corners[1].x);
      func(filter_states[3], marker_corners[1].y);
      func(filter_states[4], marker_corners[2].x);
      func(filter_states[5], marker_corners[2].y);
      func(filter_states[6], marker_corners[3].x);
      func(filter_states[7], marker_corners[3].y);
    }

    static FilterStates init_filter_states(CVFType &cv, std::vector<cv::Point2f> &marker_corners)
    {
      FilterStates fs{};
      for_each(fs, marker_corners, [&cv](CVFType::FilterState &filter_state, CVFType::ValueType &measurement) -> void
      {
        cv.init_filter_state(filter_state, measurement);
      });
      return fs;
    }

  public:
    MarkerFilter(CVFType &cv, std::vector<cv::Point2f> &marker_corners) :
      observed_{true}, // constructed because a new marker was observed.
      frames_skipped_{0},
      filter_states_{init_filter_states(cv, marker_corners)}
    {}

    auto &observed()
    { return observed_; } //
    auto &frames_skipped()
    { return frames_skipped_; } //

    std::vector<cv::Point2f> predict_only(CVFType &cv,
                                          int id)
    {
      (void)id;
      std::vector<cv::Point2f> marker_corners{cv::Point2f{}, cv::Point2f{}, cv::Point2f{}, cv::Point2f{}};
      for_each(filter_states_, marker_corners,
               [&cv](CVFType::FilterState &filter_state, CVFType::ValueType &measurement) -> void
               {
                 measurement = cv.predict(filter_state)(0);
               });
      return marker_corners;
    }

    void predict_and_update(CVFType &cv, std::vector<cv::Point2f> &marker_corners)
    {
      for_each(filter_states_, marker_corners,
               [&cv](CVFType::FilterState &filter_state, CVFType::ValueType &measurement) -> void
               {
                 cv.predict(filter_state);
                 measurement = cv.update(filter_state, measurement)(0);
               });
    }
  };

// ==============================================================================
// SmoothObservationsImpl class
// ==============================================================================

  class SmoothObservationsImpl : public SmoothObservationsInterface
  {
    const VlocContext &cxt_;
    rclcpp::Time last_time_stamp_{0, 0};
    CVFType cv_{};
    std::map<int, MarkerFilter> marker_filters_{};

  public:
    SmoothObservationsImpl(const VlocContext &cxt) :
      cxt_{cxt}
    {}

    void smooth_observations(std::vector<std::vector<cv::Point2f>> &aruco_corners,
                             std::vector<int> &aruco_ids,
                             const rclcpp::Time &time_stamp) override
    {
      // Check for conditions where we don't do smoothing.
      if (cxt_.loc_corner_filter_measure_std_ == 0.0 ||
          cxt_.loc_corner_filter_process_std_ == 0.0 ||
          last_time_stamp_.nanoseconds() == 0) {
        last_time_stamp_ = time_stamp;
        return;
      }

      // Initialize the shared filter state
      float dt = (time_stamp - last_time_stamp_).seconds();
      last_time_stamp_ = time_stamp;
      cv_.init(dt, cxt_.loc_corner_filter_process_std_, cxt_.loc_corner_filter_measure_std_);

      // Clear out the visited flag on all the filters. At the end of
      // this routine all non-visited elements are moved ahead by the filter
      // and potentially added back into the observations list.
      clear_visited();

      // Walk through the observations updating the corner locations with the filtered
      // locations.
      for (size_t i = 0; i < aruco_ids.size(); i += 1) {
        auto marker_id = aruco_ids[i];
        auto &marker_corners = aruco_corners[i];

        // Have we seen this filter before?
        auto it = marker_filters_.find(marker_id);
        if (it != marker_filters_.end()) {
          // A marker we have been following and filtering. So update
          // the corner locations with the filtered location.
          it->second.predict_and_update(cv_, marker_corners);
          it->second.observed() = true;
          it->second.frames_skipped() = 0;

        } else {
          // A marker we have not seen before. Add it to the list.
          marker_filters_.emplace(marker_id, MarkerFilter{cv_, marker_corners});
        }
      }

      // All of the markers that were observed have been processed. Now walk through
      // the filters and remove filters for markers that have not been observed.
      for (auto it = marker_filters_.begin(); it != marker_filters_.end();) {
        // If this marker was observed this frame, then leave it alone.
        if (it->second.observed()) {
          ++it;
          continue;
        }

        // This marker was not observed in this frame. If there have been too many
        // skipped frames, then remove this filter from the map.
        it->second.frames_skipped() += 1;
        if (it->second.frames_skipped() >= cxt_.loc_corner_filter_max_skipped_) {
          it = marker_filters_.erase(it);
          continue;
        }

        // This marker was not observed in this frame, but it was observed recently.
        // Add it back in the observation list with the predicted location.
        aruco_ids.emplace_back(it->first);
        aruco_corners.emplace_back(it->second.predict_only(cv_, it->first));
        ++it;
      }
    }

  private:
    void clear_visited()
    {
      for (auto &marker_filter_pair : marker_filters_) {
        marker_filter_pair.second.observed() = false;
      }
    }

  };

  std::unique_ptr<SmoothObservationsInterface> make_smooth_observations(const VlocContext &cxt)
  {
    return std::make_unique<SmoothObservationsImpl>(cxt);
  }
}
