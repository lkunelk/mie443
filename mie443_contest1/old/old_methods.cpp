
double get_angle_of_furthest_distance(int num_sectors, Move move, bool ignore_back){
    //Returns angle to rotate from current position toward the direction of furthest distance
    double furthest_distance = -1;
    double angle_of_furthest_distance = -1;
    
    double curr_angle;

    // Scanning the surrounding of the robot
    for(int i = 1; i <= num_sectors; i++) {
        // Rotate and scan
        move.rotate(DEG2RAD(360) / num_sectors, SCAN_ROT_SPEED);
        curr_angle = i * DEG2RAD(360) / num_sectors;
        ros::spinOnce();

        // Analyze the laser distance and decide if its is a candidate direction to go
        if(minLaserDist > furthest_distance && minLaserDist < 1000 
            && (!(RAD2DEG(curr_angle) < 180 + RADIUS_BACKWARD 
                && RAD2DEG(curr_angle) > 180 - RADIUS_BACKWARD) || !ignore_back)){ 
            //Second condition to ignore inf. 
            //Third condition to avoid going back the way it came.
            
            furthest_distance = minLaserDist;
            angle_of_furthest_distance = curr_angle;
        }
    }

    // Post-processing of the angle
    if (angle_of_furthest_distance == -1){
        // If no angles gave valid distances, go back the way it cam
        angle_of_furthest_distance = DEG2RAD(180); 
    }
    else if(angle_of_furthest_distance > DEG2RAD(180)){
        //e.g 270 degrees -> -90 degrees
        angle_of_furthest_distance = angle_of_furthest_distance - DEG2RAD(360); 
    }

    return angle_of_furthest_distance;
}