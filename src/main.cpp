#include <RayTracer/RayTracer.h>

int main() {
	RayTracer tracer(800, 400);
	tracer.setCamera(50, 50, 0, 0, 0, 0, 50);
	tracer.loadModel("obj/Sting-Sword-lowpoly.obj");
	tracer.addLighting(-1000, 100, 1000,
					   1000, 100, 1000,
					   0, 100, -1000,
					   1.0f, 1.0f, 1.0f);

	//tracer.loadModel("obj/Plastic_Cup-(Wavefront OBJ).obj");
	tracer.setMaxRecursionDepth(4);
	tracer.setDiffuseRayNum(4);
	tracer.render(1);
}