
#include "fiducial_math.hpp"

#include "observation.hpp"
#include "opencv2/core.hpp"

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
    void init(float dt,
              float process_std_dev,
              float measurement_std_dev); //
    void init_filter_state(FilterState &fs,
                           float measurement_0); //
  };

  void CVFType::init(float dt,
                     float process_std_dev,
                     float measurement_std_dev)
  {
    transition_matrix = {1, dt, 0, 1};
    measurement_matrix = {1, 0};
    process_noise_cov = StateCovType{dt * dt, dt, dt, 1.} * process_std_dev * process_std_dev;
    measurement_noise_cov = MeasurementCovType{measurement_std_dev * measurement_std_dev};
  }

  void CVFType::init_filter_state(FilterState &fs,
                                  float measurement_0)
  {
    fs.x_k = StateType{measurement_0, 0.}; // (phi, delta_phi)
    fs.error_cov_k = {10., 10., 10., 10.}; // Large uncertainty to start with.
  }

// ==============================================================================
// MarkerFilter class
// ==============================================================================

  class MarkerFilter
  {
    std::array<CVFType::FilterState, 8> cvf_filters_{};

    template<typename Func>
    void for_each(const Observation &observation, Func func)
    {
      func(cvf_filters_[0], observation.x0());
      func(cvf_filters_[1], observation.y0());
      func(cvf_filters_[2], observation.x1());
      func(cvf_filters_[3], observation.y1());
      func(cvf_filters_[4], observation.x2());
      func(cvf_filters_[5], observation.y2());
      func(cvf_filters_[6], observation.x3());
      func(cvf_filters_[7], observation.y3());
    }

  public:
    MarkerFilter() = default;

    void init_filter_states(CVFType &cv, const Observation &observation)
    {
      for_each(observation, [&cv](CVFType::FilterState &filter_state, float measurement) -> void
      {
        cv.init_filter_state(filter_state, measurement);
      });
    }

    void predict_only(CVFType &cv)
    {
      for (auto &filter_state : cvf_filters_) {
        cv.predict(filter_state);
      }
    }

    void predict_and_update(CVFType &cv, const Observation &observation)
    {
      for_each(observation, [&cv](CVFType::FilterState &filter_state, float measurement) -> void
      {
        cv.predict(filter_state);
        cv.update(filter_state, measurement);
      });
    }
  };

// ==============================================================================
// SmoothObservationsImpl class
// ==============================================================================

  class SmoothObservationsImpl : public SmoothObservationsInterface
  {
  public:
    SmoothObservationsImpl(const FiducialMathContext &cxt)
    {}

    void smooth_observations(Observations &observations) override
    {
      CVFType::FilterState fs;
    }

  };

  std::unique_ptr<SmoothObservationsInterface> make_smooth_observations(const FiducialMathContext &cxt)
  {
    return std::make_unique<SmoothObservationsImpl>(cxt);
  }

}
