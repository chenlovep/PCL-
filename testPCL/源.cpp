/*#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl\io\ply_io.h>
#include<pcl/point_types.h>
#include<pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>//可视化头文件
#include<pcl\visualization\cloud_viewer.h>
#include<pcl\point_cloud.h>
#include<pcl\point_representation.h>
#include<pcl\console\time.h>
#include<pcl\filters\filter.h>
#include <iostream>
#include <string>
#include <boost/make_shared.hpp>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include<pcl\filters\voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/ndt.h>      //NDT(正态分布)配准类头文件
#include <pcl/filters/approximate_voxel_grid.h>   //滤波类头文件  （使用体素网格过滤器处理的效果比较好）

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>//包含fpfh加速计算的omp多核并行计算

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
using namespace std;*/


#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);
#include<vtkPLYReader.h>
#include<vtkPLYWriter.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyData.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOrientationMarkerWidget.h> //坐标系交互
#include<vtkMatrix4x4.h>
#include<vtkMath.h>
#include<pcl\point_types.h>
#include<pcl\point_cloud.h>

//点云距离计算

int main()
{
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	//vtkSmartPointer <vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
	reader->SetFileName("skul_face.ply");
	reader->Update();
	vtkSmartPointer<vtkPLYReader> reader_1 = vtkSmartPointer<vtkPLYReader>::New();
	reader_1->SetFileName("skul3.ply");
	reader_1->Update();



	//构造浮动数据点集

	vtkSmartPointer<vtkPolyData> orig = reader->GetOutput();
	
	double dBounds[6];

	orig->GetBounds(dBounds);
	std::cout << "Xmin=" << dBounds[0] << std::endl;
	std::cout << "Xmax=" << dBounds[1] << std::endl;
	std::cout << "Ymin=" << dBounds[2] << std::endl;
	std::cout << "Ymax=" << dBounds[3] << std::endl;
	std::cout << "Zmin=" << dBounds[4] << std::endl;
	std::cout << "Zmax=" << dBounds[5] << std::endl;

	

	vtkSmartPointer<vtkTransform> trans =
		vtkSmartPointer<vtkTransform>::New();
	trans->Translate(0,0.1, 0.1);
	trans->RotateX(10);

	double dBounds_1[6];
	

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter1 =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter1->SetInputData(reader->GetOutput());
	transformFilter1->SetTransform(trans);
	transformFilter1->Update();
	
	//源数据 与 目标数据
	vtkSmartPointer<vtkPolyData> source =
		vtkSmartPointer<vtkPolyData>::New();
	source->SetPoints(orig->GetPoints());

	vtkSmartPointer<vtkPolyData> target =
		vtkSmartPointer<vtkPolyData>::New();
	target->SetPoints(reader_1->GetOutput()->GetPoints());

	vtkSmartPointer<vtkVertexGlyphFilter>  sourceGlyph =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	sourceGlyph->SetInputData(source);
	sourceGlyph->Update();

	vtkSmartPointer<vtkVertexGlyphFilter>  targetGlyph =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	targetGlyph->SetInputData(target);
	targetGlyph->Update();
	
	//进行ICP配准求变换矩阵
	vtkSmartPointer<vtkIterativeClosestPointTransform> icptrans =
		vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
	icptrans->SetSource(sourceGlyph->GetOutput());
	icptrans->SetTarget(targetGlyph->GetOutput());
	icptrans->GetLandmarkTransform()->SetModeToRigidBody();
	icptrans->SetMaximumNumberOfIterations(5000);
	icptrans->SetCheckMeanDistance(5);
	icptrans->SetMaximumMeanDistance(5e-7);
	icptrans->StartByMatchingCentroidsOn();
	icptrans->Modified();
	std::cout << "scroe" << icptrans->GetMeanDistance() << std::endl;
	icptrans->Update();

	
	//以下是新加的
	vtkMatrix4x4*M = icptrans->GetMatrix();
	std::cout << "The resulting matrix is:" << *M  << std::endl;
	for (int i = 0; i < 4; i++)
	{
		printf("\n");
		for (int j = 0; j < 4; j++)
		{
			printf("%e\t", M->Element[i][j]);
		}
	}

	//配准矩阵调整源数据
	vtkSmartPointer<vtkTransformPolyDataFilter> solution =
		vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	solution->SetInputData(sourceGlyph->GetOutput());
	solution->SetTransform(icptrans);
	solution->Update();
	/////////////////////////////////////////////////////////////
	vtkSmartPointer<vtkPolyDataMapper> sourceMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	sourceMapper->SetInputConnection(sourceGlyph->GetOutputPort());
	vtkSmartPointer<vtkActor> sourceActor =
		vtkSmartPointer<vtkActor>::New();
	sourceActor->SetMapper(sourceMapper);
	sourceActor->GetProperty()->SetColor(0, 0, 0);
	sourceActor->GetProperty()->SetPointSize(2);
	///////////////////////////
	const float voxel_grid_size = 0.005f;
	vtkSmartPointer<vtkPolyDataMapper> targetMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	targetMapper->SetInputConnection(targetGlyph->GetOutputPort());
	vtkSmartPointer<vtkActor> targetActor =
		vtkSmartPointer<vtkActor>::New();
	targetActor->SetMapper(targetMapper);
	targetActor->GetProperty()->SetColor(0, 1, 0);
	targetActor->GetProperty()->SetPointSize(3);

	vtkSmartPointer<vtkPolyDataMapper> soluMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	soluMapper->SetInputConnection(solution->GetOutputPort());
	vtkSmartPointer<vtkActor> soluActor =
		vtkSmartPointer<vtkActor>::New();
	soluActor->SetMapper(soluMapper);
	soluActor->GetProperty()->SetColor(1, 0, 0);
	soluActor->GetProperty()->SetPointSize(2);

	//vtkIterativeClosestPointTransform::GetMeanDistance(sourceGlyph);
	std::cout << "  Precision is  " << icptrans->GetMeanDistance() << std::endl;
	//设置坐标系
	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	///////////////////////////////////////////////////////////
	vtkSmartPointer<vtkRenderer> render =
		vtkSmartPointer<vtkRenderer>::New();
	render->AddActor(sourceActor);
	render->AddActor(targetActor);
	render->AddActor(soluActor);
	render->SetBackground(0, 0, 0);

	vtkSmartPointer<vtkRenderWindow> rw =
		vtkSmartPointer<vtkRenderWindow>::New();
	rw->AddRenderer(render);
	rw->SetSize(480, 480);
	rw->SetWindowName("Regisration by ICP");

	vtkSmartPointer<vtkRenderWindowInteractor> rwi =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	rwi->SetRenderWindow(rw);
	
	vtkSmartPointer<vtkOrientationMarkerWidget> widget =
		vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	widget->SetOutlineColor(1, 1, 1);
	widget->SetOrientationMarker(axes);
	widget->SetInteractor(rwi); //加入鼠标交互  
	widget->SetViewport(0.0, 0.0, 0.1, 0.1);  //设置显示位置  
	widget->SetEnabled(1);
	widget->InteractiveOn();//开启鼠标交互  
							
	render->ResetCamera();
	rw->Render();
	rwi->Initialize();
	rwi->Start();

	return 0;
}

