
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


double forward_scan(int num_sectors, Move move, double(*pick_method)(int, int, double*), bool _){
    // Returns angle to rotate from current position toward the direction of furthest distance
    // This time, it is based on the surroundings sectors as well.
    // Also, it will only sweep out the required areas.
    // NOTE: The sector number will now divide only the sweeped area, not the entire 360
    double distances[num_sectors];
    double furthest_distance = -1; 
    double angle_of_furthest_distance = -1;
    
    double curr_angle;
    double dist;

    int prev_i, next_i;

    double sweep_angle = DEG2RAD(360 - 2 * RADIUS_BACKWARD);
    double sector_size = sweep_angle / num_sectors;

    // Rotate to start position
    move.rotate(sweep_angle / 2 + sector_size, ROT_SPEED);

    // Rotate back the other way to scan to get the minLaserDist around the robot
    for(int i = 0; i < num_sectors + 2; i++) { // Extra two sectors to account for edges
        move.rotate(sector_size, -SCAN_ROT_SPEED);
        ros::spinOnce();
        distances[i] = minLaserDist; // First and last elements are not considered as candidates
    }

    // Analyze the distances and pick direction to go
    for (int i = 1; i < num_sectors + 1; i++){
        // Calculating the angle of the sector wrt original orientation
        curr_angle = sweep_angle / 2 - i * sector_size;

        // Compute the neighbourhood average and see if it's a candidate direction
        dist = (*pick_sector)(i, num_sectors, distances);
        if (dist > furthest_distance && dist < 1000){
            furthest_distance = dist;

            // Calculate angle needed to get to desired angle
            // Currently robot is located at -(sweep_angle / 2 + sector_size) wrt original orientation
            angle_of_furthest_distance = curr_angle + sweep_angle / 2 + sector_size;
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

void rotate_sim(){
    // Testing the new rotate ---------------------------------------------
    double curr_yaw = DEG2RAD(355);

    double angle = DEG2RAD(-5);

    int direction = SIGN(angle);
    bool wrap = false;
    double yaw_converted;
    double start_yaw;
    double next_angle;

    next_angle = curr_yaw + angle;
    start_yaw = curr_yaw;
    if (next_angle >= DEG2RAD(360)){
        next_angle = next_angle - DEG2RAD(360);
        wrap = true;
    }
    else if (next_angle < 0){
        next_angle = next_angle + DEG2RAD(360);
        wrap = true;
    }


    for (int i = 0; i < 20; i++){
        if (wrap && curr_yaw * direction >= start_yaw * direction){
            yaw_converted = curr_yaw - direction * DEG2RAD(360);
        }
        else{
            yaw_converted = curr_yaw;
        }


        ROS_INFO("yaw: %f, converted: %f, next: %f", RAD2DEG(curr_yaw), RAD2DEG(yaw_converted), RAD2DEG(next_angle));

        // Simulating real yaw
        curr_yaw = curr_yaw + direction * DEG2RAD(1);
        if (curr_yaw >= DEG2RAD(360)){
            curr_yaw = curr_yaw - DEG2RAD(360);
        }
        else if (curr_yaw < 0){
            curr_yaw = curr_yaw + DEG2RAD(360);
        }
        if (!(yaw_converted * direction < next_angle * direction)){
            break;
        }
    }
}