#include "BouguetInterface.h"
#include "planecalib/Map.h"
#include "planecalib/CameraModel.h"
#include "planecalib/HomographyEstimation.h"
#include <opencv2/calib3d.hpp>
#include "planecalib/HomographyCalibration.h"
#include "planecalib/PnpEstimation.h"

namespace planecalib
{

std::unique_ptr<Map> BouguetInterface::loadCalib(const std::string &filename)
// void BouguetInterface::loadCalib(const std::string &filename)
{
	//Vars to read
	int imageCount;
	Eigen::Vector2f fc, cc, kc;
	Eigen::Vector2i imageSize;
	std::vector<Eigen::Matrix2Xf> imagePoints;
	std::vector<Eigen::Matrix3f> homographies;

	// mat_t *matFile;
	// matFile = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
	// if (!matFile)
	// {
	// 	MYAPP_LOG << "Error opening mat file: " << filename << "\n";
	// 	return NULL;
	// }

	// matvar_t *matVar;
	// double *matData;

	// //Read number of images
	// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// matVar = Mat_VarRead(matFile, "n_ima");
	// if (!matVar)
	// 	throw std::runtime_error("Variable not found in mat file.");
	// matData = static_cast<double*>(matVar->data);
	// imageCount = (int)matData[0];
	imageCount = 10;
	MYAPP_LOG << "Reading calib info for " << imageCount << " images...";
	// Mat_VarFree(matVar);


	// //Read image width
	// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// matVar = Mat_VarRead(matFile, "nx");
	// if (!matVar)
	// 	throw std::runtime_error("Variable not found in mat file.");
	// matData = static_cast<double*>(matVar->data);
	// imageSize[0] = (int)matData[0];
	imageSize[0] = 1920;
	// Mat_VarFree(matVar);

	// //Read image height
	// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// matVar = Mat_VarRead(matFile, "ny");
	// if (!matVar)
	// 	throw std::runtime_error("Variable not found in mat file.");
	// matData = static_cast<double*>(matVar->data);
	// imageSize[1] = (int)matData[0];
	imageSize[1] = 1080;
	// Mat_VarFree(matVar);

	// //Read calib data
	// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// matVar = Mat_VarRead(matFile, "fc");
	// if (!matVar)
	// 	throw std::runtime_error("Variable not found in mat file.");
	// matData = static_cast<double*>(matVar->data);
	// fc[0] = (float)matData[0];
	// fc[1] = (float)matData[1];
	fc[0] = 1759.95830447;
	fc[1] = 1758.22389526;
	// Mat_VarFree(matVar);

	// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// matVar = Mat_VarRead(matFile, "cc");
	// if (!matVar)
	// 	throw std::runtime_error("Variable not found in mat file.");
	// matData = static_cast<double*>(matVar->data);
	// cc[0] = (float)matData[0];
	// cc[1] = (float)matData[1];
	cc[0] = 963.80029367;
	cc[1] = 541.04810506;
	// Mat_VarFree(matVar);

	// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// matVar = Mat_VarRead(matFile, "kc");
	// if (!matVar)
	// 	throw std::runtime_error("Variable not found in mat file.");
	// matData = static_cast<double*>(matVar->data);
	// kc[0] = (float)matData[0];
	// kc[1] = (float)matData[1];
	kc[0] = 0.16261665;
	kc[1] = -0.67445366;
	// Mat_VarFree(matVar);

	// //Read image data
	imagePoints.resize(imageCount);
	homographies.resize(imageCount);
	std::cout << "Loading points ..." << std::endl;
	for (int i = 0; i < imageCount; i++)
	{
		//Image points
		{
			// std::stringstream ss;
			// ss << "x_" << (i + 1);
			// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
			// matVar = Mat_VarRead(matFile, ss.str().c_str());
			// if (!matVar)
			// 	throw std::runtime_error("Variable not found in mat file.");
			// assert(matVar->rank == 2);
			// assert(matVar->dims[0] == 2);
			// assert(matVar->data_type == MAT_T_DOUBLE);
			// assert(matVar->class_type == MAT_C_DOUBLE);

			Eigen::Matrix2Xd points(2, 77);
			points << 1542.1104709173119, 1551.8072430137033, 1562.0190499562289, 1572.559667979691, 1582.8016279264248, 1592.8621279188967, 1603.2233355077199, 1430.8320518217859, 1438.262162562929, 1447.5339339583304, 1456.7202978328407, 1465.770548231782, 1475.3212891917408, 1484.4923624189742, 1316.929575684852, 1323.4477220474107, 1330.446401579545, 1337.9723684412493, 1345.7770186388739, 1354.3038750376563, 1363.2659541207556, 1201.5621305300717, 1206.9121536578689, 1212.695852503474, 1218.8983841209554, 1225.5476510274302, 1233.0369615267512, 1240.6560884217777, 1085.6193604035557, 1089.99660738269, 1094.8722003438033, 1099.9575956550186, 1105.2820986777544, 1111.319697764026, 1117.4414786954771, 968.364082774059, 972.066240991225, 975.6787147890682, 979.7046869453634, 983.7786545711782, 987.8607805541894, 991.9088422608394, 850.0903953725066, 852.7544449097625, 855.5464508821931, 858.1172847557101, 860.6579619644099, 863.4703700863058, 866.1570544097152, 730.4183357959865, 732.0075621981936, 733.7755413270875, 735.0309409967413, 736.1074837089652, 737.4021654554342, 738.5058618715419, 608.8137190041988, 608.9816849770036, 609.3076441872159, 609.3927368155761, 609.1967284209169, 608.5404875686343, 608.1276797005818, 485.9641980807456, 484.7174510542863, 483.0918299740151, 481.8026182593706, 480.1391850858846, 478.19806533435536, 476.4969817621186, 362.57377313832654, 359.4359264142559, 356.21110269802165, 353.065066075643, 350.5310579846895, 346.9694230281645, 344.77151293908673, 149.37327901400994, 259.90391727960383, 372.42393104968204, 487.56695737540326, 605.7720204777823, 726.4124801634902, 849.2696399525992, 148.51748306912617, 260.8155904303195, 373.36980320069085, 489.1946205426913, 608.1206223494895, 729.3353207903634, 853.6219403749143, 148.1843895820364, 260.8772693840621, 374.90284764498375, 491.192406212875, 609.9802024294387, 732.2631764545052, 856.8319294999295, 149.00331651825923, 262.11594486822395, 376.69044233576335, 493.2433933630248, 612.7470375552332, 734.675007012243, 860.3516565595412, 149.28197118087456, 263.0250947049864, 377.88500271851046, 494.99956474871675, 615.0436437613419, 737.5891748480051, 863.4572607908333, 149.4171076978505, 263.09815934256125, 378.7825674569082, 496.7086123121633, 617.2552309796205, 740.7729369491982, 867.654661321172, 149.18045674719215, 264.05211050294514, 379.78856442742915, 498.37704057718815, 619.5525104744105, 744.1722524500212, 871.674815155682, 148.84556828920188, 264.29132839345436, 381.0875142278585, 500.37485414663433, 622.5818029606127, 747.5483495447339, 876.7330293674565, 147.9026182351793, 263.85812686758953, 381.9568372960291, 502.34365086629697, 625.4733712687623, 752.1868865610501, 881.3615765808337, 147.24742231601815, 264.1122597326334, 382.67641039960404, 504.06525881397124, 628.6509194204851, 755.9947513359655, 886.4943758220631, 147.55502712096091, 264.6679077129881, 383.9648036341611, 506.4741456185945, 631.543701871365, 759.9088525558008, 890.2438594475913;

			// points.resize(2, 77);
			// points = x_1;
			// memcpy(points.data(), &x_1, sizeof(x_1));
			// for (int x = 0; x < 77; x++) {
			// 	for (int y = 0; y < 2; y++) {
			// 		points << x_1[x][y],
			// 	}
			// }
			imagePoints[i] = points.cast<float>();
			// Mat_VarFree(matVar);
		}

	// 	//Homography
		{
	// 		std::stringstream ss;
	// 		ss << "H_" << (i + 1);
	// 		Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// 		matVar = Mat_VarRead(matFile, ss.str().c_str());
	// 		if (!matVar)
	// 			throw std::runtime_error("Variable not found in mat file.");
	// 		assert(matVar->rank == 2);
	// 		assert(matVar->dims[0] == 3 && matVar->dims[1] == 3);
	// 		assert(matVar->data_type == MAT_T_DOUBLE);
	// 		assert(matVar->class_type == MAT_C_DOUBLE);
			Eigen::Matrix3d h;
			h << -776.2166210114891, 13958.449378885432, 367.92232530284133, 12537.375479455506, 108.83091169982298, 151.2643771822424, -1.2308480778669575, 0.6303114556007764, 1.0;
	// 		memcpy(h.data(), matVar->data, matVar->nbytes);
	// 		homographies[i] = h.cast<float>();
	// 		Mat_VarFree(matVar);
		}
	}

	// Mat_Close(matFile);

	// //Create map
	int featureCount = imagePoints[0].cols();
	std::cout << "imagePoints 1: " << imagePoints[1] << std::endl;
	std::unique_ptr<Map> map(new Map);

	//Create features
	for (int i = 0; i < featureCount; i++)
	{
		std::unique_ptr<Feature> feature(new Feature);
		feature->setPosition(imagePoints[0].col(i));
		map->addFeature(std::move(feature));
	}

	// //Create keyframes
	cv::Mat3b nullImg3(imageSize[1], imageSize[0]);
	cv::Mat1b nullImg1(imageSize[1], imageSize[0]);
	Eigen::Matrix<uchar, 1, 32> nullDescr;
	nullDescr.setZero();
	Eigen::Matrix3f refHinv = homographies[0].inverse();
	for (int k = 0; k< imageCount; k++)
	{
		std::unique_ptr<Keyframe> frame(new Keyframe);

		frame->init(nullImg3, nullImg1);
		frame->setPose(homographies[k] * refHinv);

		for (int i = 0; i < featureCount; i++)
		{
			std::unique_ptr<FeatureMeasurement> m(new FeatureMeasurement(map->getFeatures()[i].get(), frame.get(), imagePoints[k].col(i), 0, nullDescr.data()));
			frame->getMeasurements().push_back(m.get());
			map->getFeatures()[i]->getMeasurements().push_back(std::move(m));
		}

		map->addKeyframe(std::move(frame));
	}

	std::unique_ptr<CameraModel> camera( new CameraModel());
	camera->init(fc, cc, imageSize);
	camera->getDistortionModel().init(camera->getMaxRadiusSq());
	// throw std::logic_error("Not implemented");

	map->mCamera = std::move(camera);
	return std::move(map);
}

void BouguetInterface::loadValidation(const CameraModel &camera, const std::string &filename)
{
	// //Vars to read
	// int imageCount;
	// std::vector<Eigen::Matrix2Xf> imagePoints;
	// std::vector<Eigen::Matrix3Xf> worldPoints;

	// mat_t *matFile;
	// matFile = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
	// if (!matFile)
	// {
	// 	MYAPP_LOG << "Error opening mat file: " << filename << "\n";
	// 	return NULL;
	// }

	// matvar_t *matVar;
	// double *matData;

	// //Read number of images
	// Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// matVar = Mat_VarRead(matFile, "n_ima");
	// if (!matVar)
	// 	throw std::runtime_error("Variable not found in mat file.");
	// matData = static_cast<double*>(matVar->data);
	// imageCount = (int)matData[0];
	// MYAPP_LOG << "Reading valid info for " << imageCount << " images...";
	// Mat_VarFree(matVar);


	// //Read image data
	// imagePoints.resize(imageCount);
	// worldPoints.resize(imageCount);
	// for (int i = 0; i < imageCount; i++)
	// {
	// 	//Image points
	// 	{
	// 		std::stringstream ss;
	// 		ss << "x_" << (i + 1);
	// 		Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// 		matVar = Mat_VarRead(matFile, ss.str().c_str());
	// 		if (!matVar)
	// 			throw std::runtime_error("Variable not found in mat file.");
	// 		assert(matVar->rank == 2);
	// 		assert(matVar->dims[0] == 2);
	// 		assert(matVar->data_type == MAT_T_DOUBLE);
	// 		assert(matVar->class_type == MAT_C_DOUBLE);

	// 		Eigen::Matrix2Xd points;
	// 		points.resize(2, matVar->dims[1]);
	// 		memcpy(points.data(), matVar->data, matVar->nbytes);
	// 		imagePoints[i] = points.cast<float>();
	// 		Mat_VarFree(matVar);
	// 	}

	// 	//World points
	// 	{
	// 		std::stringstream ss;
	// 		ss << "X_" << (i + 1);
	// 		Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	// 		matVar = Mat_VarRead(matFile, ss.str().c_str());
	// 		if (!matVar)
	// 			throw std::runtime_error("Variable not found in mat file.");
	// 		assert(matVar->rank == 2);
	// 		assert(matVar->dims[0] == 3);
	// 		assert(matVar->data_type == MAT_T_DOUBLE);
	// 		assert(matVar->class_type == MAT_C_DOUBLE);

	// 		Eigen::Matrix3Xd points;
	// 		points.resize(3, matVar->dims[1]);
	// 		memcpy(points.data(), matVar->data, matVar->nbytes);
	// 		worldPoints[i] = points.cast<float>();
	// 		Mat_VarFree(matVar);
	// 	}
	// }

	// Mat_Close(matFile);

	// //Check that all world points are the same
	// for (int i = 1; i < worldPoints.size(); i++)
	// {
	// 	if (!worldPoints[0].array().isApprox(worldPoints[i].array(), 0.0001))
	// 		MYAPP_LOG << "ERROR: World points are not equal for all images!!!!\n";
	// }

	// //Create map
	// int featureCount = imagePoints[0].cols();
	// std::unique_ptr<Map> map(new Map);

	// //Create features
	// for (int i = 0; i < featureCount; i++)
	// {
	// 	std::unique_ptr<Feature> feature(new Feature);
	// 	feature->setPosition(imagePoints[0].col(i));
	// 	feature->mGroundTruthPosition3D = worldPoints[0].col(i);
	// 	feature->mPosition3D = feature->mGroundTruthPosition3D;
	// 	map->addFeature(std::move(feature));
	// }

	// //Translate to vectors
	// std::vector<Eigen::Vector3f> worldPointsVec;
	// worldPointsVec.resize(featureCount);
	// for (int i = 0; i < featureCount; i++)
	// {
	// 	worldPointsVec[i] = worldPoints[0].col(i);
	// }
	// std::vector<std::vector<Eigen::Vector2f>> imagePointsVec;
	// imagePointsVec.resize(imageCount);
	// for (int k = 0; k < imageCount; k++)
	// {
	// 	imagePointsVec[k].resize(featureCount);
	// 	for (int i = 0; i < featureCount; i++)
	// 	{
	// 		imagePointsVec[k][i] = imagePoints[k].col(i);
	// 	}
	// }

	// //Create keyframes
	// std::vector<float> scales(worldPointsVec.size(), 1);
	// cv::Mat3b nullImg3(camera.getImageSize()[1], camera.getImageSize()[0]);
	// cv::Mat1b nullImg1(camera.getImageSize()[1], camera.getImageSize()[0]);
	// Eigen::Matrix<uchar, 1, 32> nullDescr;
	// nullDescr.setZero();
	// for (int k = 0; k< imageCount; k++)
	// {
	// 	std::unique_ptr<Keyframe> frame(new Keyframe);

	// 	frame->init(nullImg3, nullImg1);

	// 	//Find pose
	// 	PnPRansac ransac;
	// 	ransac.setParams(3, 10, 100, (int)(0.9f*featureCount));
	// 	ransac.setData(&worldPointsVec, &imagePointsVec[k], &scales, &camera);
	// 	ransac.doRansac();
	// 	frame->mPose3DR = ransac.getBestModel().first.cast<float>();
	// 	frame->mPose3DT = ransac.getBestModel().second.cast<float>();
	// 	//frame->setPose(homographies[k] * refHinv);

	// 	for (int i = 0; i < featureCount; i++)
	// 	{
	// 		std::unique_ptr<FeatureMeasurement> m(new FeatureMeasurement(map->getFeatures()[i].get(), frame.get(), imagePointsVec[k][i], 0, nullDescr.data()));
	// 		frame->getMeasurements().push_back(m.get());
	// 		map->getFeatures()[i]->getMeasurements().push_back(std::move(m));
	// 	}

	// 	map->addKeyframe(std::move(frame));
	// }

	// return std::move(map);
}

}
