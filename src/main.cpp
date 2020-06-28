#include <RayTracer/RayTracer.h>

int main() {
	RayTracer tracer(400, 200);
	tracer.loadModel("test");
	tracer.renderOneFrame();
	tracer.outputImg();
}