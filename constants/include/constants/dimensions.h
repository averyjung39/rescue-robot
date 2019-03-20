namespace dimensions {
    // Dimensions related to the map
    const float TILE_WIDTH_CM = 30.48;
    const float HALF_TILE_WIDTH_CM = TILE_WIDTH_CM / 2.0;
    const float MAP_WIDTH = TILE_WIDTH_CM * 6;
    const float MAP_HEIGHT = TILE_WIDTH_CM * 6;

    // Dimensions related to robot
    // Offsets are in cm and from the center of the robot
    // TODO: remeasure these
    const float BOTTOM_TOF_Y_OFFSET = 1.905;
    const float BOTTOM_TOF_X_OFFSET = 5.08;
    const float TOP_TOF_FRONT_X_OFFSET = 5.08;
    const float TOP_TOF_FRONT_Y_OFFSET = 0.0;
    
    const float TOP_TOF_OFFSET = 10.16;
    const float TOP_TOF_LEFT_X_OFFSET = -0.9525;
    const float TOP_TOF_RIGHT_X_OFFSET = 0.3175;
    const float TOP_TOF_BACK_Y_OFFSET = -0.635;
};