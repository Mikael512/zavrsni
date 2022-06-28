#ifndef ZAVRSNI_PROGRAM2_H
#define ZAVRSNI_PROGRAM2_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <zavrsni/example.h>

namespace zavrsni
{
/**
 * @brief Basic example leveraging trajopt and tesseract for cartesian planning
 */
class Program2 : public Example
{
public:
  Program2(tesseract_environment::Environment::Ptr env,
                        tesseract_visualization::Visualization::Ptr plotter = nullptr,
                        bool ifopt = false,
                        bool debug = false);

  ~Program2() override = default;
  Program2(const Program2&) = default;
  Program2& operator=(const Program2&) = default;
  Program2(Program2&&) = default;
  Program2& operator=(Program2&&) = default;

  tesseract_common::JointTrajectory run() override final;

private:
  bool ifopt_;
  bool debug_;
  static tesseract_environment::Command::Ptr addPointCloud();
};

}  // namespace tesseract_examples

#endif  // ZAVRSNI_PROGRAM2_H
