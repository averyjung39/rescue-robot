namespace dimensions {
    // Dimensions related to the map
    const float TILE_WIDTH_CM = 30.48;
    const float HALF_TILE_WIDTH_CM = TILE_WIDTH_CM / 2.0;
    const float MAP_WIDTH = TILE_WIDTH_CM * 6;
    const float MAP_HEIGHT = TILE_WIDTH_CM * 6;

    // Dimensions related to robot
    // Offsets are in cm and from the center of the robot
    // TODO: remeasure these
    const float BOTTOM_TOF_LEFT_Y_OFFSET = 3.33375;
    const float BOTTOM_TOF_RIGHT_Y_OFFSET = -3.65125;
    const float FRONT_TOF_X_OFFSET = 10.16;
    const float TOP_TOF_FRONT_Y_OFFSET = -7.62;

    const float TOP_TOF_Y_OFFSET = 10.16;
    const float TOP_TOF_LEFT_X_OFFSET = -2.2225;
    const float TOP_TOF_RIGHT_X_OFFSET = -2.06375;
};
