#ifndef CONFIG_H_
#define CONFIG_H_

#define COLOR_RED Scalar(0,0,255)
#define COLOR_GREEN Scalar(0,255,0)
#define COLOR_BLUE Scalar(255,0,0)
#define COLOR_YELLOW Scalar(0,255,255)

#if 1

#define YML_GENERATED_PTS2D3D_FLOW "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/ptsForDtInFlow.yml"
//#define YML_GENERATED_PTS2D3D_FLOW "input/GeneratedTriangles/Mon_9_Nov_2020_14-10-40-generated_triangles_in_motion.yml"
#define YML_PARAMETERS_LEVMAR "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/parameters_bUseAzToInit.yml"
#define YML_EXTRNSICS "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/userdefextrinsics.yml" 
#define YML_INTRINSICS "input/GeneratedDtInMotion/Thu_1_Apr_2021_04-45-56/userdefintrinsics.yml" 

#else

#define YML_GENERATED_PTS2D3D_FLOW "input/GeneratedDtInMotion/Fri_23_Apr_2021_00-16-52/ptsForDtInFlow.yml"
#define YML_PARAMETERS_LEVMAR ""
#define YML_EXTRNSICS "input/GeneratedDtInMotion/Fri_23_Apr_2021_00-16-52/userdefextrinsics.yml" 
#define YML_INTRINSICS "input/GeneratedDtInMotion/Fri_23_Apr_2021_00-16-52/userdefintrinsics.yml" 

#endif

#endif // CONFIG_H_