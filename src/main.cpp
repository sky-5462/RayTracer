#include <RayTracer/RayTracer.h>

int main() {
	RayTracer tracer(800, 400);
	tracer.setCamera(50, 50, 0, 0, 0, 0, 50);
	tracer.loadModel("obj/Sting-Sword-lowpoly.obj");
	tracer.addLighting(-500, 100, 500,
					   500, 100, 500,
					   0, 100, -500,
					   1.0f, 1.0f, 1.0f);

	//tracer.loadModel("obj/Plastic_Cup-(Wavefront OBJ).obj");
	tracer.setMaxRecursionDepth(2);
	tracer.setDiffuseRayNum(4);
	tracer.render(20);
}