#include <RayTracer/RayTracer.h>
#include <fstream>
#include <iostream>
#include <string>

int main(int args, char** argv) {
	if (args != 2) {
		std::cout << "Invalid arguments\n";
		return -100;
	}
	std::ifstream config(argv[1]);
	std::string key;
	int width, height;
	if (config >> key && key == "frame")
		config >> width >> height;
	else {
		std::cout << "Error: can't parse \"frame\"\n";
		return -1;
	}

	RayTracer tracer(width, height);

	if (config >> key && key == "setting") {
		while (config >> key) {
			if (key == "camera") {
				float x, y, z, atX, atY, atZ, focal, rotate;
				config >> x >> y >> z >> atX >> atY >> atZ >> focal >> rotate;
				tracer.setCamera(x, y, z, atX, atY, atZ, focal, rotate);
			}
			else if (key == "background_color") {
				float r, g, b;
				config >> r >> g >> b;
				tracer.setBackgroundColor(r, g, b);
			}
			else if (key == "max_recursion_depth") {
				int depth;
				config >> depth;
				tracer.setMaxRecursionDepth(depth);
			}
			else if (key == "diffuse_ray_number") {
				int diffuseNum;
				config >> diffuseNum;
				tracer.setDiffuseRayNum(diffuseNum);
			}
			else if (key == "specular_ray_number") {
				int specularNum;
				config >> specularNum;
				tracer.setSpecularRayNum(specularNum);
			}
			else if (key == "end_setting")
				break;
		}
	}
	else {
		std::cout << "Error: can't parse \"setting\"\n";
		return -2;
	}

	int modelNum;
	if (config >> key && key == "model_num")
		config >> modelNum;
	else {
		std::cout << "Error: can't parse \"model_num\"\n";
		return -3;
	}

	for (int i = 0; i < modelNum; ++i) {
		float offsetX, offsetY, offsetZ;
		if (config >> key && key == "position_offset")
			config >> offsetX >> offsetY >> offsetZ;
		else {
			std::cout << "Error: can't parse \"position_offset\"\n";
			return -4;
		}

		std::string path;
		if (config >> key && key == "model_path")
			config >> path;
		else {
			std::cout << "Error: can't parse \"model_path\"\n";
			return -5;
		}

		tracer.loadModel(path, i, offsetX, offsetY, offsetZ);

		while (config >> key) {
			if (key == "texture_path") {
				config >> path;
				tracer.loadTexture(path, i);
			}
			else if (key == "override_color") {
				float r, g, b;
				config >> r >> g >> b;
				tracer.overrideColor(i, r, g, b);
			}
			else if (key == "override_is_metal") {
				bool isMetal;
				config >> isMetal;
				tracer.overrideIsMetal(i, isMetal);
			}
			else if (key == "override_is_light_emitting") {
				bool isLightEmitting;
				config >> isLightEmitting;
				tracer.overrideIsLightEmitting(i, isLightEmitting);
			}
			else if (key == "override_is_transparent") {
				bool isTransparent;
				config >> isTransparent;
				tracer.overrideIsTransparent(i, isTransparent);
			}
			else if (key == "override_specularRoughness") {
				float roughness;
				config >> roughness;
				tracer.overrideSpecularRoughness(i, roughness);
			}
			else if (key == "override_refractive_index") {
				float refIndex;
				config >> refIndex;
				tracer.overrideRefractiveIndex(i, refIndex);
			}
			else if (key == "model_end")
				break;
		}
	}

	int triangleNum;
	if (config >> key && key == "extra_triangles_num")
		config >> triangleNum;
	else {
		std::cout << "Error: can't parse \"extra_triangles_num\"\n";
		return -6;
	}

	for (int i = 0; i < triangleNum; ++i) {
		float v0x, v0y, v0z;
		if (config >> key && key == "vertex_0")
			config >> v0x >> v0y >> v0z;
		else {
			std::cout << "Error: can't parse \"vertex_0\"\n";
			return -7;
		}

		float v1x, v1y, v1z;
		if (config >> key && key == "vertex_1")
			config >> v1x >> v1y >> v1z;
		else {
			std::cout << "Error: can't parse \"vertex_1\"\n";
			return -8;
		}

		float v2x, v2y, v2z;
		if (config >> key && key == "vertex_2")
			config >> v2x >> v2y >> v2z;
		else {
			std::cout << "Error: can't parse \"vertex_2\"\n";
			return -9;
		}

		float nx, ny, nz;
		if (config >> key && key == "normal_side")
			config >> nx >> ny >> nz;
		else {
			std::cout << "Error: can't parse \"normal_side\"\n";
			return -10;
		}

		float r = 1, g = 1, b = 1;
		bool isMetal = false, isLightEmitting = false, isTransparent = false;
		float specularRoughness = 0, refIndex = 1.5;
		while (config >> key) {
			if (key == "color")
				config >> r >> g >> b;
			else if (key == "is_metal")
				config >> isMetal;
			else if (key == "is_light_emitting")
				config >> isLightEmitting;
			else if (key == "is_transparent")
				config >> isTransparent;
			else if (key == "specularRoughness")
				config >> specularRoughness;
			else if (key == "refractive_index")
				config >> refIndex;
			else if (key == "end_triangle")
				break;
		}

		tracer.addTriangle(v0x, v0y, v0z,
						   v1x, v1y, v1z,
						   v2x, v2y, v2z,
						   nx, ny, nz,
						   r, g, b,
						   isMetal, isLightEmitting, isTransparent,
						   specularRoughness, refIndex);
	}

	if (config >> key && key == "render") {
		int renderNum;
		config >> renderNum;
		tracer.render(renderNum);
	}
	else {
		std::cout << "Error: can't parse \"render\"\n";
		return -12;
	}
}