#include <iostream>
#include <fstream>
//g++ -I /usr/local/Cellar/eigen/3.3.3/include/eigen3/ *.cpp -o my_program -std=c++11 -lfreeimage

#include "Eigen.h"

#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;
	//nVertices = 5;
	//std::cout << sizeof(vertices[0]) << std::endl;
	// TODO: Get number of faces
	unsigned nFaces = 0;




	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;


	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(unsigned int i = 0; i < nVertices; ++i)
	{
		if (vertices[i].position(0)==MINF)
			outFile << 0 << ' ' << 0 << ' '<< 0<<' '<< 0;
		else
			outFile << vertices[i].position(0) << ' ' << vertices[i].position(1) << ' '<< vertices[i].position(2)<<' '<<vertices[i].position(3);
		//std::cout <<i<<" "<< vertices[i].position(0) << ' ' << vertices[i].position(1) << ' '<< vertices[i].position(2);
		outFile <<' '<< (unsigned int)vertices[i].color(0) << ' ' << (unsigned int)vertices[i].color(1) << ' '<< (unsigned int)vertices[i].color(2)<<' '<< (unsigned int)vertices[i].color(3);
		outFile << std::endl;
	}




	// TODO: save faces






	// close file
	outFile.close();

	return true;
}

int main()
{
	std::string filenameIn = "data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "result/mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();
		//std::cout << (unsigned int)colorMap[0] << std::endl;
		//break;

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		for (unsigned int v = 0; v < sensor.GetDepthImageHeight(); ++v)
			for(unsigned int u = 0; u < sensor.GetDepthImageWidth(); ++u)
			{
				unsigned int idx = v * sensor.GetDepthImageWidth()+u;
				float z = depthMap[idx];
				if (z == MINF)
				{
					//std::cout<<idx<<" " << "MINF !!!!"<<std::endl;

					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0,0,0,0);
				}
				else
				{
					float x = (u+1-cX) * z / fovX;
					float y = (v+1-cY) * z / fovY;
					vertices[idx].position = trajectoryInv * Vector4f(x,y,z,1.0);
					//std::cout <<idx<<" "<< vertices[idx].position<<std::endl;
					vertices[idx].color = Vector4uc(colorMap[idx*4],colorMap[idx*4+1],colorMap[idx*4+2],colorMap[idx*4+3]);
					//std::cout <<idx<<" "<< vertices[idx].color<<std::endl;
				}
			}

			//for (int i = 0; i < 5; ++i)
			//	std::cout<<vertices[i].position<<std::endl;






		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
		
	}

	return 0;
}
