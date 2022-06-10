#ifndef ZAVRSNI_PROGRAM1_H
#define ZAVRSNI_PROGRAM1_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <zavrsni/example.h>

namespace zavrsni
{
/**
 * @brief An example of a robot with fixed orientation but free to move in cartesian space
 * leveraging tesseract and trajopt to generate a motion trajectory.
 */
class Program1 : public Example
{
public:
  Program1(tesseract_environment::Environment::Ptr env,
                      tesseract_visualization::Visualization::Ptr plotter = nullptr,
                      bool ifopt = false,
                      bool debug = false);
  ~Program1() override = default;
  Program1(const Program1&) = default;
  Program1& operator=(const Program1&) = default;
  Program1(Program1&&) = default;
  Program1& operator=(Program1&&) = default;

  tesseract_common::JointTrajectory run() override final;

private:
  bool ifopt_;
  bool debug_;
  static tesseract_environment::Command::Ptr addSphere();
};

}  // namespace zavrsni

#endif  // ZAVRSNI_PROGRAM1_H
