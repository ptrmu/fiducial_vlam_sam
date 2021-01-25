#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <vector>

#include "transform3_with_covariance.hpp"

namespace fvlam
{
// ==============================================================================
// Observation class
// ==============================================================================

  class Observation
  {
  public:
    static constexpr size_t ArraySize = 4;
    using Element = Translate2;
    using Array = std::array<Element, ArraySize>;
    using MuVector = Eigen::Matrix<double, Element::MuVector::MaxRowsAtCompileTime * ArraySize, 1>;

    bool is_valid_;

    // The id of the marker that we observed.
    std::uint64_t id_;

    // The four marker corners in the image frame.
    // origin = upper left, x -> left to right, y -> top to bottom
    Array corners_f_image_;

    Element::CovarianceMatrix cov_;

  public:
    Observation() :
      is_valid_{false}, id_{0},
      corners_f_image_{Translate2(), Translate2(), Translate2(), Translate2()},
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    explicit Observation(std::uint64_t id) :
      is_valid_{false}, id_{id},
      corners_f_image_{Translate2(), Translate2(), Translate2(), Translate2()},
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    Observation(std::uint64_t id, Array corners_f_image) :
      is_valid_{true}, id_(id),
      corners_f_image_(std::move(corners_f_image)),
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    Observation(std::uint64_t id, Array corners_f_image, Element::CovarianceMatrix cov) :
      is_valid_{true}, id_(id),
      corners_f_image_(std::move(corners_f_image)),
      cov_{std::move(cov)}
    {}

    Observation(std::uint64_t id,
                double x0, double y0,
                double x1, double y1,
                double x2, double y2,
                double x3, double y3) :
      is_valid_{true}, id_(id),
      corners_f_image_{Element{x0, y0}, Element{x1, y1}, Element{x2, y2}, Element{x3, y3}},
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    auto is_valid() const
    { return is_valid_; }

    auto id() const
    { return id_; }

    auto &corners_f_image() const
    { return corners_f_image_; }

    template<class T>
    static Observation from(T &other);

    template<class T>
    static Observation from(std::uint64_t id, T &other);

    // If the type we are converting to is an array, a vector, or matrix, then the to() routines extract
    // the corners_f_image_ values and put them in the array.
    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const Observation &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };

// ==============================================================================
// Observations class
// ==============================================================================

  class Observations
  {
    std::uint64_t stamp_;

    // The list of observations
    std::vector<Observation> observations_{};

  public:
    explicit Observations(std::uint64_t stamp = 0) :
      stamp_(stamp)
    {}

    auto const &observations() const
    { return observations_; }

    auto size() const
    { return observations_.size(); }

    template<typename T>
    static Observations from(T &other);

    template<typename T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const Observations &other, double tol = 1.0e-9, bool check_relative_also = true) const;

    void add(const Observation &observation)
    {
      observations_.emplace_back(observation);
    }
  };


}
