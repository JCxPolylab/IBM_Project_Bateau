//Execution sur rapsi en remote + récupération érreurs : timeout 10s ./CATJ_ibmRobotProject; echo "exit=$?"

#include "main.h"

#define DIR_NAME "IBM_robot_SimuPC"

// Dir raspi Jerry
/*
#define DIR_NAME_LINUX "IBM_Bateau"
*/

//Dir raspi EPF
#define DIR_NAME_LINUX "jerryCamera"


#ifdef WIN32
#include <windows.h>
#endif

//D�claration des variables
bool flagRecording = false;
bool showImage = false;
int vKey = 0;
std::string imgCodec;
std::string imgExt;

int main() 
{
    // Instanciation des classes
    std::cout << "démarrage du programme" << std::endl;
    CATJ_utility::iniReader iniFile;
	std::filesystem::path PrjPath = CATJ_utility::executable_dir();
	CATJ_utility::programme_mode mode = CATJ_utility::programme_mode::unknown;
    CATJ_error::error erreur;
    cv::Mat frame;
    cv::Mat calibOut;
    UINT32 colorTargetHex = 0, colorIgnoreHex = 0;
    std::vector<cv::Point3f> objTemplate;
    std::vector<std::vector<cv::Point3f>> objPts;
    std::string str;
    MeasureState measTools;
    std::vector<CATJ_camera::BallDetection> dets;

    //D�claration des variables
    bool debugMode = false;
    int valA, valB, valC = 0;
    float fvalA = 0.0f, fvalB = 0.0f;
    int cvKey = 0;
    bool ballDetected = false;


#ifdef WIN32

	// On remonte les dossiers jusqu'� trouver le dossier du projet 
    while(PrjPath.empty() || PrjPath.filename() != DIR_NAME) {
        if (PrjPath.empty()) {
            std::cerr << "Erreur : Impossible de trouver le r�pertoire du projet." << std::endl;
            return -10;
        }
        PrjPath = PrjPath.parent_path();
	}
#else
    // On remonte les dossiers jusqu'� trouver le dossier du projet 
    while (PrjPath.empty() || PrjPath.filename() != DIR_NAME_LINUX) {
        if (PrjPath.empty()) {
            std::cerr << "Erreur : Impossible de trouver le r�pertoire du projet." << std::endl;
            return -10;
        }
        PrjPath = PrjPath.parent_path();
    }
#endif

    /*********************************************************************************************/
    /*                                  Configuration                                            */    
    /*********************************************************************************************/
	//Chargement du fichier de configuration


    
    std::cout << "Récupération des paramètres dans le fichier .ini" << std::endl;
    check_negzerror_ret(iniFile.load((PrjPath / "parametres" / "config.ini").string()), "dada");

	//r�cuperation des donn�es de config
	iniFile.get("GENERAL", "debug flag", debugMode);
	iniFile.get("GENERAL", "programme mode", str);
	mode = CATJ_utility::strToMode(str);

    //Camera
	iniFile.get("CAMERA", "reference", str);
    iniFile.get("CAMERA", "device", valA);
    iniFile.get("CAMERA", "width", valB);
    iniFile.get("CAMERA", "height", valC);
    CATJ_camera::Camera cam(valA, valB, valC);
	iniFile.get("CAMERA", "fps", valA);
    cam.setFps(valA);
    iniFile.get("CAMERA", "Aifps", valA);
    cam.setAiFps(valA); 
    iniFile.get("CAMERA", "calibration file", str);
	cam.setCalibPath(PrjPath.string() + "/" + str);
    iniFile.get("CAMERA", "echiquier size", str);
	cam.setEchiquierSizeCalibration({ std::stoi(str.substr(0, str.find(","))), std::stoi(str.substr((str.find(",") + 1))) });
    iniFile.get("CAMERA", "square size", fvalA);
	cam.setSquareSizeCalibration(fvalA);
    iniFile.get("CAMERA", "flag recording", flagRecording);
	cam.setRecording(flagRecording);
    iniFile.get("CAMERA", "show image", showImage);
    cam.setShow(showImage);
    iniFile.get("CAMERA", "non maximum suppression threshold", fvalA);
    iniFile.get("CAMERA", "score threshold", fvalB);
	cam.setThresholds(fvalB, fvalA);
    iniFile.get("CAMERA", "target color", str);
    CATJ_utility::xstoi(str, colorTargetHex);
    iniFile.get("CAMERA", "ignore color", colorIgnoreHex);
    CATJ_utility::xstoi(str, colorIgnoreHex);
	iniFile.get("CAMERA", "hardware focal mm", fvalA);
	cam.setHWFocaleMm(fvalA);
    iniFile.get("CAMERA", "nombres de prise pour la calibration", valA);
	cam.setNeededViews(valA);
    iniFile.get("CAMERA", "video codec", imgCodec);
    iniFile.get("CAMERA", "video ext", imgExt);

    iniFile.get("IA", "ONNX model path", str);
    std::filesystem::path modelPath = std::filesystem::path(str).is_absolute()
        ? std::filesystem::path(str)
        : (PrjPath / str);
    check_negzerror_ret(std::filesystem::exists(modelPath), "ONNX introuvable: " + modelPath.string());
    cam.loadCalibration(cam.getcalibPath());
    cam.loadBallDetectorONNX(modelPath.string(), cam.getOnnx_InputSize());

    switch (mode)
    {
        /*************************** CAMERA *********************************/
        case CATJ_utility::programme_mode::camera:
            std::cout << "Mode CAMERA\n";
            cam.startCapture(60.0);

            if (showImage)
                cam.setShow(true);
            if (flagRecording)
                cam.startRecording("debug." + std::string(imgExt), 20.0, imgCodec);

            // Chargement du mod�le onnx (traitement IA)
            //cam.loadBallDetectorONNX("ball.onnx", 320);
            cam.setThresholds(0.35f, 0.45f);
            cam.setAIFps(5);
            cam.setColorDecision(CATJ_camera::BallColor::Red, CATJ_camera::BallColor::Blue);

            
            while (cam.isOpen()) 
            {
                if (cam.detectBalls(dets))
                {
					std::cout << "Balle d�tect�e !\n" << std::endl;
                }
            }
		break;

        /*************************** CALIBRATION *********************************/
        case CATJ_utility::programme_mode::calibration:
            cam.startCapture(60.0);
            cam.setShow(true);

			std::cout << "Mode CALIBRATION\n";

            objTemplate = cam.makeChessboard3D(cam.getEchiquierSizeCalibration(),
                cam.getSquareSizeCalibration());

            objPts = { objTemplate };

			std::cout << "Appuyez sur la touche 'q' pour quitter la programmation\n" << std::endl;
            while (cam.getKeyPolled() != 'q')
            {
                std::cout << cam.getKeyPolled() << std::endl;
                while (!cam.getLatestFrame(frame)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                if (cam.calibrateCamera(true, objTemplate, frame, &calibOut)) {
                    std::cout << "Calibration r�ussie !\n";
                    cv::imwrite(PrjPath.string() + "/calibration_result.jpg", calibOut);
                    break;
                }
                else {
                    std::cerr << "Calibration �chou�e.\n";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
        break;

        /*************************** COURSE *********************************/
        case CATJ_utility::programme_mode::course:
        std::cout << "Mode COURSE\n";
        break;

        /*************************** RAMASSAGE *********************************/
        case CATJ_utility::programme_mode::ramassage:
        std::cout << "Mode RAMASSAGE\n";
        break;

        /*************************** LIDAR *********************************/
        case CATJ_utility::programme_mode::lidar:
        std::cout << "Mode LIDAR\n";
        break;

        /*************************** COLORSENSOR *********************************/
        case CATJ_utility::programme_mode::colorsensor:
        std::cout << "Mode COLORSENSOR\n";
        break;

        /*************************** GYRO *********************************/
        case CATJ_utility::programme_mode::gyro:
        std::cout << "Mode GYRO\n";
        break;

        /*************************** NAVIGATION *********************************/
        case CATJ_utility::programme_mode::navigation:
        std::cout << "Mode NAVIGATION\n";
        break;

        /*************************** REMOTE *********************************/
        case CATJ_utility::programme_mode::remote:
        std::cout << "Mode REMOTE\n";
        break;

        case CATJ_utility::programme_mode::mesure:
        std::cout << "Mode MESURE\n";

        cam.startCapture(cam.getFps());
        measTools.drawing = false;
        measTools.hasLine = false;
        measTools.realSizeMm = 10;
        cam.runMeasureTool(cam, measTools);

        std::cout << "camera status" << cam.isOpen() << std::endl;
        while (cam.isOpen())
        {
            std::cout << "Dernière mesure : " << measTools.lastPx << " px, distance estimée : " << measTools.lastDistMm << " mm\n";  
        }
        break;

        case CATJ_utility::programme_mode::debug:
        std::cout << "Mode DEBUG/capture loop\n";

        cam.startCapture(cam.getFps());
        cam.startRecording("debug." + imgExt, 20.0, imgCodec);

        if (flagRecording)

		std::cout << "Appuyez sur la touche 'q' pour quitter le programme\n" << std::endl;

        while (cam.isOpen())
        {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (cv::pollKey() == 'q')
            {
                cam.stopCapture();
                break;
            }
        }

        break;

        /*************************** UNKNOW *********************************/
		case CATJ_utility::programme_mode::unknown:
            set_error(-20 , "Mode inconnu dans le config.ini\n");

        /*************************** DEFAULT *********************************/
        default:
            set_error(-30, "Mode inconnu dans le config.ini\n");
	}

    return 0;


err:    //Execution du programme en cas d'erreur


    return erreur.code;
}
