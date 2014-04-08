#include <raytracer/Image.hpp>
#include <raytracer/PerspectiveCamera.hpp>
#include <raytracer/Scene.hpp>
#include <raytracer/Raytracer.hpp>
#include <raytracer/Light.hpp>
#include <raytracer/ConstantMaterial.hpp>
#include <raytracer/Plane.hpp>
#include <raytracer/Math.hpp>
#include <opengl/RaytracerWindow.hpp>

bool gShowWindow=true;
int imageWidth=512;
int imageHeight=512;
std::string gDataPath= ""; ///< The path pointing to the resources (OBJ, shader)

// Duplicate this function and play around with the configuration!
std::shared_ptr<rt::Scene> makeTestScene()
{
  //The scene holds all renderable objects, lights and a camera.
  std::shared_ptr<rt::Scene>    scene     = std::make_shared<rt::Scene>();

  //Create a perspective camera, looking at the origin (0,0,0).
  //The up-direction is (0,0,1).
  std::shared_ptr<rt::Camera>   camera    = std::make_shared<rt::PerspectiveCamera>();

  //Move the camera eye position to (5,0,5).
  camera->setPosition(rt::Vec3d(5,0,5));

  //Set Horizontal and Vertical field of view to 60 degrees.
  camera->setFOV(60,60);
  scene->setCamera(camera);

  //The scene needs a light for tracing rays, the intensity does not matter though.
  //The point light source is positioned at (5,2,6)
  std::shared_ptr<rt::Light>    light1     = std::make_shared<rt::Light>(rt::Vec3d(5,2,6), rt::Vec3d(1,1,1));
  scene->addLight(light1);

  //Create several materials: orange (spheres), blue (plane), red (triangle).
  std::shared_ptr<rt::Material> materialPlane    = std::make_shared<rt::ConstantMaterial>(rt::Vec3d(0.0,0.2,0.7));

  // This plane is initialized with default values.
  // point on plane: (0,0,0)
  // plane normal: (0,0,1)
  std::shared_ptr<rt::Plane> plane = std::make_shared<rt::Plane>();
  plane->setMaterial(materialPlane);
  scene->addRenderable(plane);

  return scene;
}

int main(int argc, char** argv)
{
  std::string pathToThisFile=__FILE__;

  if(gDataPath.empty())
    gDataPath=rt::Math::getParentDirectoryFromFilePath(pathToThisFile);
  std::cerr<<"Using data path: "<<gDataPath<<std::endl;

  std::shared_ptr<rt::Image> image = std::make_shared<rt::Image>(imageWidth,imageHeight);
  std::shared_ptr<rt::Scene> scene = makeTestScene();

  std::shared_ptr<rt::Raytracer> raytracer = std::make_shared<rt::Raytracer>();
  raytracer->setScene(scene);

  rt::Chrono before = std::chrono::high_resolution_clock::now();
  raytracer->renderToImage(image);
  rt::ChronoDuration timeToRender = std::chrono::duration_cast<rt::ChronoDuration>(std::chrono::high_resolution_clock::now()-before);
  std::cout<<"Rendering took "<<timeToRender.count()<<" seconds"<<std::endl;

  image->saveToTGA(std::string(argv[0])+"_result");

  image->saveToTGA(std::string(argv[0])+"_result");

  if(gShowWindow)
  {
    rt::RaytracerWindow win(imageWidth,imageHeight);
    if(win.init())
      win.drawOnce(image);
    else
      return -1;
  }
  return 0;
}
