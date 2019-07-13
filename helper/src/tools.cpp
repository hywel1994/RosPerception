#include <helper/tools.h>

Tools::Tools(bool is_kitti){
	if (is_kitti){
		// Fill transformation matrices
		TRANS_VELO_TO_CAM = MatrixXf::Zero(4, 4);
		TRANS_VELO_TO_CAM << 
			7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03,  
			1.480249000000e-02,  7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02,  
			9.998621000000e-01,  7.523790000000e-03,  1.480755000000e-02, -2.717806000000e-01,
			0, 0 ,0 ,0;


		TRANS_CAM_TO_RECTCAM = MatrixXf::Zero(4, 4);
		TRANS_CAM_TO_RECTCAM << 
			9.999239000000e-01, 9.837760000000e-03, -7.445048000000e-03, 0,
			-9.869795000000e-03, 9.999421000000e-01, -4.278459000000e-03, 0,
			7.402527000000e-03, 4.351614000000e-03,  9.999631000000e-01, 0,
			0, 0, 0, 1;


		TRANS_RECTCAM_TO_IMAGE = MatrixXf::Zero(3, 4);
		TRANS_RECTCAM_TO_IMAGE << 
			7.215377000000e+02, 0.000000000000e+00, 6.095593000000e+02, 4.485728000000e+01,
			0.000000000000e+00, 7.215377000000e+02, 1.728540000000e+02, 2.163791000000e-01,
			0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 2.745884000000e-03;

		SEMANTIC_NAMES = std::vector<std::string>{
			// Static objects
			"Road", "Sidewalk", "Building", "Wall", "Fence", "Pole",
			"Traffic light", "Traffic sign", "Vegetation", "Terrain", "Sky",
			// Dynamic objects
			"Pedestrian", "Rider", "Car", "Truck", "Bus", "Train", "Motocycle", "Bicycle"
		};

		SEMANTIC_COLOR_TO_CLASS = std::map<int, int>{
			// Static objects
			{320, 0}, {511, 1}, {210, 2}, {260, 3}, {496, 4}, {459, 5},
			{450, 6}, {440, 7}, {284, 8}, {555, 9}, {380, 10},
			// Dynamic objects
			{300, 11}, {255, 12}, {142, 13}, {70, 14},{160, 15}, {180, 16}, {230, 17}, {162, 18}
		};

		SEMANTIC_CLASS_TO_COLOR = MatrixXi::Zero(19, 3);
		SEMANTIC_CLASS_TO_COLOR <<
		// Static objects
			128,  64, 128, // Road
			244,  35, 232, // Sidewalk
			70,  70,  70, // Building
			102, 102, 156, // Wall
			190, 153, 153, // Fence
			153, 153, 153, // Pole
			250, 170,  30, // Traffic light
			220, 220,   0, // Traffic sign
			107, 142,  35, // Vegetation
			152, 251, 152, // Terrain
			70, 130, 180, // Sky

		// Dynamic objects
			220,  20,  60, // Pedestrian
			255,   0,   0, // Rider
			0,   0, 142, // Car
			0,   0,  70, // Truck
			0,  60, 100, // Bus
			0,  80, 100, // Train
			0,   0, 230, // Motocycle
			119,  11,  32;  // Bicycle

		SEMANTIC_KERNEL_SIZE = VectorXi::Zero(8);	
		SEMANTIC_KERNEL_SIZE <<
			1, // Pedestrian
			2, // Rider
			1, // Car
			4, // Truck
			5, // Bus
			5, // Train
			2, // Motocycle
			2; // Bicycle
	}
	else{
	// Fill transformation matrices
		TRANS_VELO_TO_CAM = MatrixXf::Zero(4, 4);
		TRANS_VELO_TO_CAM << 
			0, -1,  0, 0,
			0,  0, -1, 1.5,
			1,  0,  0, 0,  // camera height
			0,  0,  0, 0;


		TRANS_CAM_TO_RECTCAM = MatrixXf::Zero(4, 4);
		TRANS_CAM_TO_RECTCAM << 
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,1;

		TRANS_RECTCAM_TO_IMAGE = MatrixXf::Zero(3, 4);
		TRANS_RECTCAM_TO_IMAGE << 
			816.118268598647, 0, 680.6511245884145, 0,
			0, 822.0196620588329, 458.230641061779, 0,
			0, 0, 1, 0;


		SEMANTIC_NAMES = std::vector<std::string>{
			"wall",
			"building;edifice",
			"sky",
			"floor;flooring",
			"tree",
			"ceiling",
			"road;route",
			"bed",
			"windowpane;window",
			"grass",
			"cabinet",
			"sidewalk;pavement",
			"person;individual;someone;somebody;mortal;soul",
			"earth;ground",
			"door;double;door",
			"table",
			"mountain;mount",
			"plant;flora;plant;life",
			"curtain;drape;drapery;mantle;pall",
			"chair",
			"car;auto;automobile;machine;motorcar",
			"water",
			"painting;picture",
			"sofa;couch;lounge",
			"shelf",
			"house",
			"sea",
			"mirror",
			"rug;carpet;carpeting",
			"field",
			"armchair",
			"seat",
			"fence;fencing",
			"desk",
			"rock;stone",
			"wardrobe;closet;press",
			"lamp",
			"bathtub;bathing;tub;bath;tub",
			"railing;rail",
			"cushion",
			"base;pedestal;stand",
			"box",
			"column;pillar",
			"signboard;sign",
			"chest;of;drawers;chest;bureau;dresser",
			"counter",
			"sand",
			"sink",
			"skyscraper",
			"fireplace;hearth;open;fireplace",
			"refrigerator;icebox",
			"grandstand;covered;stand",
			"path",
			"stairs;steps",
			"runway",
			"case;display;case;showcase;vitrine",
			"pool;table;billiard;table;snooker;table",
			"pillow",
			"screen;door;screen",
			"stairway;staircase",
			"river",
			"bridge;span",
			"bookcase",
			"blind;screen",
			"coffee;table;cocktail;table",
			"toilet;can;commode;crapper;pot;potty;stool;throne",
			"flower",
			"book",
			"hill",
			"bench",
			"countertop",
			"stove;kitchen;stove;range;kitchen;range;cooking;stove",
			"palm;palm;tree",
			"kitchen;island",
			"computer;computing;machine;computing;device;data;processor;electronic;computer;information;processing;system",
			"swivel;chair",
			"boat",
			"bar",
			"arcade;machine",
			"hovel;hut;hutch;shack;shanty",
			"bus;autobus;coach;charabanc;double-decker;jitney;motorbus;motorcoach;omnibus;passenger;vehicle",
			"towel",
			"light;light;source",
			"truck;motortruck",
			"tower",
			"chandelier;pendant;pendent",
			"awning;sunshade;sunblind",
			"streetlight;street;lamp",
			"booth;cubicle;stall;kiosk",
			"television;television;receiver;television;set;tv;tv;set;idiot;box;boob;tube;telly;goggle;box",
			"airplane;aeroplane;plane",
			"dirt;track",
			"apparel;wearing;apparel;dress;clothes",
			"pole",
			"land;ground;soil",
			"bannister;banister;balustrade;balusters;handrail",
			"escalator;moving;staircase;moving;stairway",
			"ottoman;pouf;pouffe;puff;hassock",
			"bottle",
			"buffet;counter;sideboard",
			"poster;posting;placard;notice;bill;card",
			"stage",
			"van",
			"ship",
			"fountain",
			"conveyer;belt;conveyor;belt;conveyer;conveyor;transporter",
			"canopy",
			"washer;automatic;washer;washing;machine",
			"plaything;toy",
			"swimming;pool;swimming;bath;natatorium",
			"stool",
			"barrel;cask",
			"basket;handbasket",
			"waterfall;falls",
			"tent;collapsible;shelter",
			"bag",
			"minibike;motorbike",
			"cradle",
			"oven",
			"ball",
			"food;solid;food",
			"step;stair",
			"tank;storage;tank",
			"trade;name;brand;name;brand;marque",
			"microwave;microwave;oven",
			"pot;flowerpot",
			"animal;animate;being;beast;brute;creature;fauna",
			"bicycle;bike;wheel;cycle",
			"lake",
			"dishwasher;dish;washer;dishwashing;machine",
			"screen;silver;screen;projection;screen",
			"blanket;cover",
			"sculpture",
			"hood;exhaust;hood",
			"sconce",
			"vase",
			"traffic;light;traffic;signal;stoplight",
			"tray",
			"ashcan;trash;can;garbage;can;wastebin;ash;bin;ash-bin;ashbin;dustbin;trash;barrel;trash;bin",
			"fan",
			"pier;wharf;wharfage;dock",
			"crt;screen",
			"plate",
			"monitor;monitoring;device",
			"bulletin;board;notice;board",
			"shower",
			"radiator",
			"glass;drinking;glass",
			"clock",
			"flag"

		};

		SEMANTIC_COLOR_TO_CLASS_VECTOR = std::map<std::vector<int>, int>{
				{{120, 120, 120}, 0},
				{{180, 120, 120}, 1},
				{{6, 230, 230}, 2},
				{{80, 50, 50}, 3},
				{{4, 200, 3}, 4},
				{{120, 120, 80}, 5},
				{{140, 140, 140}, 6},
				{{204, 5, 255}, 7},
				{{230, 230, 230}, 8},
				{{4, 250, 7}, 9},
				{{224, 5, 255}, 10},
				{{235, 255, 7}, 11},
				{{150, 5, 61}, 12},
				{{120, 120, 70}, 13},
				{{8, 255, 51}, 14},
				{{255, 6, 82}, 15},
				{{143, 255, 140}, 16},
				{{204, 255, 4}, 17},
				{{255, 51, 7}, 18},
				{{204, 70, 3}, 19},
				{{0, 102, 200}, 20},
				{{61, 230, 250}, 21},
				{{255, 6, 51}, 22},
				{{11, 102, 255}, 23},
				{{255, 7, 71}, 24},
				{{255, 9, 224}, 25},
				{{9, 7, 230}, 26},
				{{220, 220, 220}, 27},
				{{255, 9, 92}, 28},
				{{112, 9, 255}, 29},
				{{8, 255, 214}, 30},
				{{7, 255, 224}, 31},
				{{255, 184, 6}, 32},
				{{10, 255, 71}, 33},
				{{255, 41, 10}, 34},
				{{7, 255, 255}, 35},
				{{224, 255, 8}, 36},
				{{102, 8, 255}, 37},
				{{255, 61, 6}, 38},
				{{255, 194, 7}, 39},
				{{255, 122, 8}, 40},
				{{0, 255, 20}, 41},
				{{255, 8, 41}, 42},
				{{255, 5, 153}, 43},
				{{6, 51, 255}, 44},
				{{235, 12, 255}, 45},
				{{160, 150, 20}, 46},
				{{0, 163, 255}, 47},
				{{140, 140, 140}, 48},
				{{0250, 10, 15}, 49},
				{{20, 255, 0}, 50},
				{{31, 255, 0}, 51},
				{{255, 31, 0}, 52},
				{{255, 224, 0}, 53},
				{{153, 255, 0}, 54},
				{{0, 0, 255}, 55},
				{{255, 71, 0}, 56},
				{{0, 235, 255}, 57},
				{{0, 173, 255}, 58},
				{{31, 0, 255}, 59},
				{{11, 200, 200}, 60},
				{{255 ,82, 0}, 61},
				{{0, 255, 245}, 62},
				{{0, 61, 255}, 63},
				{{0, 255, 112}, 64},
				{{0, 255, 133}, 65},
				{{255, 0, 0}, 66},
				{{255, 163, 0}, 67},
				{{255, 102, 0}, 68},
				{{194, 255, 0}, 69},
				{{0, 143, 255}, 70},
				{{51, 255, 0}, 71},
				{{0, 82, 255}, 72},
				{{0, 255, 41}, 73},
				{{0, 255, 173}, 74},
				{{10, 0, 255}, 75},
				{{173, 255, 0}, 76},
				{{0, 255, 153}, 77},
				{{255, 92, 0}, 78},
				{{255, 0, 255}, 79},
				{{255, 0, 245}, 80},
				{{255, 0, 102}, 81},
				{{255, 173, 0}, 82},
				{{255, 0, 20}, 83},
				{{255, 184, 184}, 84},
				{{0, 31, 255}, 85},
				{{0, 255, 61}, 86},
				{{0, 71, 255}, 87},
				{{255, 0, 204}, 88},
				{{0, 255, 194}, 89},
				{{0, 255, 82}, 90},
				{{0, 10, 255}, 91},
				{{0, 112, 255}, 92},
				{{51, 0, 255}, 93},
				{{0, 194, 255}, 94},
				{{0, 122, 255}, 95},
				{{0, 255, 163}, 96},
				{{255, 153, 0}, 97},
				{{0, 255, 10}, 98},
				{{255, 112, 0}, 99},
				{{143, 255, 0}, 100},
				{{82, 0, 255}, 101},
				{{163, 255, 0}, 102},
				{{255, 235, 0}, 103},
				{{8, 184, 170}, 104},
				{{133, 0, 255}, 105},
				{{0, 255, 92}, 106},
				{{184, 0, 255}, 107},
				{{255, 0, 31}, 108},
				{{0, 184, 255}, 109},
				{{0, 214, 255}, 110},
				{{255, 0, 112}, 111},
				{{92, 255, 0}, 112},
				{{0, 224, 255}, 113},
				{{112, 224, 255}, 114},
				{{70, 184, 160}, 115},
				{{163, 0, 255}, 116},
				{{153, 0, 255}, 117},
				{{71, 255, 0}, 118},
				{{255, 0, 163}, 119},
				{{255, 204, 0}, 120},
				{{255, 0, 143}, 121},
				{{0, 255, 235}, 122},
				{{133, 255, 0}, 123},
				{{255, 0, 235}, 124},
				{{245, 0, 255}, 125},
				{{255, 0, 122}, 126},
				{{255, 245, 0}, 127},
				{{10, 190, 212}, 128},
				{{214, 255, 0}, 129},
				{{0, 204, 255}, 130},
				{{20, 0, 255}, 131},
				{{255, 255, 0}, 132},
				{{0, 153, 255}, 133},
				{{0, 41, 255}, 134},
				{{0, 255, 204}, 135},
				{{41, 0, 255}, 136},
				{{41, 255, 0}, 137},
				{{173, 0, 255}, 138},
				{{0, 245, 255}, 139},
				{{71, 0, 255}, 140},
				{{122, 0, 255}, 141},
				{{0, 255, 184}, 142},
				{{0, 92, 255}, 143},
				{{184, 255, 0}, 144},
				{{0, 133, 255}, 145},
				{{255, 214, 0}, 146},
				{{25, 194, 194}, 147},
				{{102, 255, 0}, 148},
				{{92, 0, 255}, 149}};
		
		SEMANTIC_CLASS_TO_COLOR = MatrixXi::Zero(150, 3);
		SEMANTIC_CLASS_TO_COLOR <<
				120, 120, 120,	//360	wall
				180, 120, 120,	//420	building;edifice
				6, 230, 230,	//466	sky
				80, 50, 50,		//180	floor;flooring
				4, 200, 3,		//207	tree
				120, 120, 80,	//320	ceiling
				140, 140, 140,	//420	road;route
				204, 5, 255,	//464	bed
				230, 230, 230,	//690	windowpane;window
				4, 250, 7,		//261	grass
				224, 5, 255,	//484	cabinet
				235, 255, 7,	//497	sidewalk;pavement
				150, 5, 61, 	//216	person;individual;someone;somebody;mortal;soul
				120, 120, 70,	//310	earth;ground
				8, 255, 51, 	//314	door;double;door
				255, 6, 82, 	//
				143, 255, 140, 	//
				204, 255, 4, 	//
				255, 51, 7, 	//
				204, 70, 3, 	//
				0, 102, 200, 	//
				61, 230, 250, 	//
				255, 6, 51, 	//
				11, 102, 255, 	//
				255, 7, 71, 	//
				255, 9, 224, 	//
				9, 7, 230, 		//
				220, 220, 220,	//
				255, 9, 92, 	//
				112, 9, 255, 	//
				8, 255, 214, 	//
				7, 255, 224, 	//
				255, 184, 6, 	//
				10, 255, 71, 	//
				255, 41, 10, 	//
				7, 255, 255, 	//
				224, 255, 8, 	//
				102, 8, 255, 	//
				255, 61, 6, 	//
				255, 194, 7, 	//
				255, 122, 8, 	//
				0, 255, 20, 	//
				255, 8, 41, 	//
				255, 5, 153, 	//
				6, 51, 255, 	//
				235, 12, 255, 	//
				160, 150, 20, 	//
				0, 163, 255, 	//
				140, 140, 140, 	//
				0250, 10, 15, 	//
				20, 255, 0, 	//
				31, 255, 0, 	//
				255, 31, 0, 	//
				255, 224, 0, 	//
				153, 255, 0, 	//
				0, 0, 255, 		//
				255, 71, 0, 	//
				0, 235, 255, 	//
				0, 173, 255, 	//
				31, 0, 255, 	//
				11, 200, 200, 	//
				255 ,82, 0, 	//
				0, 255, 245, 	//
				0, 61, 255, 	//
				0, 255, 112, 	//
				0, 255, 133, 	//
				255, 0, 0, 		//
				255, 163, 0, 	//
				255, 102, 0, 	//
				194, 255, 0, 	//
				0, 143, 255, 	//
				51, 255, 0, 	//
				0, 82, 255, 	//
				0, 255, 41, 	//
				0, 255, 173, 	//
				10, 0, 255, 	//
				173, 255, 0, 	//
				0, 255, 153, 	//
				255, 92, 0, 	//
				255, 0, 255, 	//
				255, 0, 245, 	//
				255, 0, 102, 	//
				255, 173, 0, 	//
				255, 0, 20, 	//
				255, 184, 184, 	//
				0, 31, 255, 	//
				0, 255, 61, 	//
				0, 71, 255, 	//
				255, 0, 204, 	//
				0, 255, 194, 	//
				0, 255, 82, 	//
				0, 10, 255, 	//
				0, 112, 255, 	//
				51, 0, 255, 	//
				0, 194, 255,	//
				0, 122, 255, 	//
				0, 255, 163, 	//
				255, 153, 0, 	//
				0, 255, 10, 	//
				255, 112, 0, 	//
				143, 255, 0, 	//
				82, 0, 255, 	//
				163, 255, 0, 	//
				255, 235, 0, 	//
				8, 184, 170, 	//
				133, 0, 255, 	//
				0, 255, 92, 	//
				184, 0, 255, 	//
				255, 0, 31, 	//
				0, 184, 255, 	//
				0, 214, 255, 	//
				255, 0, 112, 	//
				92, 255, 0, 	//
				0, 224, 255, 	//
				112, 224, 255, 	//
				70, 184, 160, 	//
				163, 0, 255, 	//
				153, 0, 255, 	//
				71, 255, 0, 	//
				255, 0, 163, 	//
				255, 204, 0, 	//
				255, 0, 143, 	//
				0, 255, 235, 	//
				133, 255, 0, 	//
				255, 0, 235,	//
				245, 0, 255,	//
				255, 0, 122, 	//
				255, 245, 0, 	//
				10, 190, 212, 	//
				214, 255, 0, 	//
				0, 204, 255, 	//
				20, 0, 255, 	//
				255, 255, 0, 	//
				0, 153, 255, 	//
				0, 41, 255, 	//
				0, 255, 204, 	//
				41, 0, 255, 	//
				41, 255, 0, 	//
				173, 0, 255, 	//
				0, 245, 255, 	//
				71, 0, 255, 	//
				122, 0, 255, 	//
				0, 255, 184, 	//
				0, 92, 255, 	//
				184, 255, 0, 	//
				0, 133, 255, 	//
				255, 214, 0, 	//
				25, 194, 194, 	//
				102, 255, 0, 	//
				92, 0, 255; 	//


		SEMANTIC_KERNEL_SIZE = VectorXi::Zero(150);	
		SEMANTIC_KERNEL_SIZE <<
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5,
			5;

	}
	
}

Tools::~Tools(){

}

int Tools::getClusterKernel(const int semantic){

	if(semantic > 10)
		return SEMANTIC_KERNEL_SIZE(semantic);
	else
		return -1;
}

MatrixXf Tools::getImage2DBoundingBox(
	const Point & point,
	const float width,
	const float height){

	MatrixXf velo_points = MatrixXf::Zero(4,2);
	velo_points(0,0) = point.x;
	velo_points(1,0) = point.y + width;
	velo_points(2,0) = point.z + height;
	velo_points(3,0) = 1;
	velo_points(0,1) = point.x;
	velo_points(1,1) = point.y - width;
	velo_points(2,1) = point.z;
	velo_points(3,1) = 1;
	MatrixXf image_points = transformVeloToImage(velo_points);
	return image_points;
}

MatrixXf Tools::getImage2DBoundingBox(
	const Object o){

	// Rotate top view box with velo orientation
	float rad_ori = o.orientation / 180 * M_PI;

	float half_length = o.length / 2;
	float half_width = o.width / 2;
	float cos_l = half_length * cos(rad_ori);
	float sin_w = half_width * sin(rad_ori);
	float sin_l = half_length * sin(rad_ori);
	float cos_w = half_width * cos(rad_ori);

	MatrixXf velo_points = MatrixXf::Zero(4,8);
	velo_points(0,0) = o.velo_pose.point.x + cos_l + sin_w;
	velo_points(1,0) = o.velo_pose.point.y + sin_l - cos_w;
	velo_points(2,0) = o.velo_pose.point.z + o.height;
	velo_points(3,0) = 1;

	velo_points(0,1) = o.velo_pose.point.x + cos_l - sin_w;
	velo_points(1,1) = o.velo_pose.point.y + sin_l + cos_w;
	velo_points(2,1) = o.velo_pose.point.z + o.height;
	velo_points(3,1) = 1;

	velo_points(0,2) = o.velo_pose.point.x - cos_l + sin_w;
	velo_points(1,2) = o.velo_pose.point.y - sin_l - cos_w;
	velo_points(2,2) = o.velo_pose.point.z + o.height;
	velo_points(3,2) = 1;

	velo_points(0,3) = o.velo_pose.point.x - cos_l - sin_w;
	velo_points(1,3) = o.velo_pose.point.y - sin_l + cos_w;
	velo_points(2,3) = o.velo_pose.point.z + o.height;
	velo_points(3,3) = 1;

	velo_points(0,4) = o.velo_pose.point.x + cos_l + sin_w;
	velo_points(1,4) = o.velo_pose.point.y + sin_l - cos_w;
	velo_points(2,4) = o.velo_pose.point.z;
	velo_points(3,4) = 1;

	velo_points(0,5) = o.velo_pose.point.x + cos_l - sin_w;
	velo_points(1,5) = o.velo_pose.point.y + sin_l + cos_w;
	velo_points(2,5) = o.velo_pose.point.z;
	velo_points(3,5) = 1;

	velo_points(0,6) = o.velo_pose.point.x - cos_l + sin_w;
	velo_points(1,6) = o.velo_pose.point.y - sin_l - cos_w;
	velo_points(2,6) = o.velo_pose.point.z;
	velo_points(3,6) = 1;

	velo_points(0,7) = o.velo_pose.point.x - cos_l - sin_w;
	velo_points(1,7) = o.velo_pose.point.y - sin_l + cos_w;
	velo_points(2,7) = o.velo_pose.point.z;
	velo_points(3,7) = 1;

	MatrixXf image_points = transformVeloToImage(velo_points);

	float min_x = image_points(0,0);
	float max_x = image_points(0,0);
	float min_y = image_points(1,0);
	float max_y = image_points(1,0);
	for(int i = 1; i < 8; i++){
		min_x = (min_x < image_points(0,i)) ? min_x : image_points(0,i);
		max_x = (max_x > image_points(0,i)) ? max_x : image_points(0,i);
		min_y = (min_y < image_points(1,i)) ? min_y : image_points(1,i);
		max_y = (max_y > image_points(1,i)) ? max_y : image_points(1,i);
	}

	// Check bounding
	if(min_x < 0)
		min_x = 0.0;
	if(max_x > 1237)
		max_x = 1237.0;
	if(min_y < 0)
		min_y = 0.0;
	if(max_y > 370)
		max_y = 370.0;

	MatrixXf box = MatrixXf::Zero(2,2);
	box(0,0) = min_x;
	box(1,0) = min_y;
	box(0,1) = max_x;
	box(1,1) = max_y;

	return box;
}

MatrixXf Tools::transformVeloToCam(const MatrixXf & velo_points){

	return TRANS_VELO_TO_CAM * velo_points;
}

MatrixXf Tools::transformCamToRectCam(const MatrixXf & cam_points){

	return TRANS_CAM_TO_RECTCAM * cam_points;
}

MatrixXf Tools::transformRectCamToImage(const MatrixXf & rect_cam_points){

	MatrixXf image_points = TRANS_RECTCAM_TO_IMAGE * rect_cam_points;
	MatrixXf uv = MatrixXf::Zero(3,rect_cam_points.cols());
	uv.row(0) = image_points.row(0).array()/image_points.row(2).array();
	uv.row(1) = image_points.row(1).array()/image_points.row(2).array();
	uv.row(2) = image_points.row(2);
	return uv;
}

MatrixXf Tools::transformVeloToImage(const MatrixXf & velo_points){

	return transformRectCamToImage(TRANS_CAM_TO_RECTCAM * TRANS_VELO_TO_CAM * velo_points);
}

