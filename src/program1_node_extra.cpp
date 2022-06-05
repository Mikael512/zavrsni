#include <zavrsni/program1.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace zavrsni;
using namespace tesseract_common;
using namespace tesseract_environment;

int main(int /*argc*/, char** /*argv*/)
{
  auto locator = std::make_shared<TesseractSupportResourceLocator>();
  tesseract_common::fs::path urdf_path =
      locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
  tesseract_common::fs::path srdf_path =
      locator->locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath();
  auto env = std::make_shared<Environment>();
  if (!env->init(urdf_path, srdf_path, locator))
    exit(1);

  Program1 example(env, nullptr);
  if (!example.run())
    exit(1);
}
