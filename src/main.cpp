#include <RayTracer/RayTracer.h>
#include <iostream>

int main(int args, char** argv) {
	if (args != 2) {
		std::cout << "Invalid arguments\n";
		return -1;
	}
	RayTracer tracer;
	try {
		tracer.parseConfigFile(argv[1]);
	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
	}
}