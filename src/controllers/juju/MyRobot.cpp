#include "MyRobot.h"
#include <iomanip>
#include <sstream>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()  // Constructor of MyRobot class, inheriting from Robot class
{
    world = 1;  // Initialize the world / Initialisation de l'environnement

    _time_step = 64;  // Time step duration / Durée du pas de temps

    _left_speed = 0;  // Left wheel speed / Vitesse de la roue gauche
    _right_speed = 0;  // Right wheel speed / Vitesse de la roue droite

    _x = _y = _theta = 0.0;  // Initial position and orientation / Position et orientation initiale
    _x_offset = _y_offset = _theta_offset = 0.0;  // Offsets for position and orientation / Décalages de position et orientation

    _sr = _sl = 0.0;  // Speed variables for left and right wheels / Variables de vitesse pour les roues gauche et droite

    _x_goal = 0.0;  // X position goal / Objectif de position X
    _y_goal = 0.0;  // Y position goal / Objectif de position Y
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  // Calculate goal orientation / Calcul de l'orientation de l'objectif

    _left_wheel_sensor = getPositionSensor("left wheel sensor");  // Left wheel position sensor / Capteur de position de la roue gauche
    _right_wheel_sensor = getPositionSensor("right wheel sensor");  // Right wheel position sensor / Capteur de position de la roue droite
    _left_wheel_sensor->enable(_time_step);  // Enable left wheel sensor / Activation du capteur de la roue gauche
    _right_wheel_sensor->enable(_time_step);  // Enable right wheel sensor / Activation du capteur de la roue droite

    _my_compass = getCompass("compass");  // Compass / Boussole
    _my_compass->enable(_time_step);  // Enable the compass / Activation de la boussole

    _gps = getGPS("gps");  // GPS / GPS
    _gps->enable(_time_step);  // Enable the GPS / Activation du GPS
    
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++) {  // Initializing distance sensors / Initialisation des capteurs de distance
        //cout << "Initializing distance sensor: " << ds_name[ind] << endl; 
        _distance_sensor[ind] = getDistanceSensor(ds_name[ind]);  // Initialize the distance sensor / Initialisation du capteur de distance
        _distance_sensor[ind]->enable(_time_step);  // Enable the distance sensor / Activation du capteur de distance
    }
    
    _forward_camera = getCamera("camera_f");  // Forward camera / Caméra avant
    _forward_camera->enable(_time_step);  // Enable the forward camera / Activation de la caméra avant

    _left_wheel_motor = getMotor("left wheel motor");  // Left wheel motor / Moteur de la roue gauche
    _right_wheel_motor = getMotor("right wheel motor");  // Right wheel motor / Moteur de la roue droite

    _right_wheel_motor->setPosition(0.0);  // Set the initial position for the right wheel motor / Position initiale du moteur de la roue droite
    _left_wheel_motor->setPosition(0.0);  // Set the initial position for the left wheel motor / Position initiale du moteur de la roue gauche

    _right_wheel_motor->setPosition(INFINITY);  // Infinite position for free-moving wheel / Position infinie pour une roue libre
    _left_wheel_motor->setPosition(INFINITY);  // Infinite position for free-moving wheel / Position infinie pour une roue libre

    _right_wheel_motor->setVelocity(0.0);  // Set initial velocity for the right wheel motor / Vitesse initiale du moteur de la roue droite
    _left_wheel_motor->setVelocity(0.0);  // Set initial velocity for the left wheel motor / Vitesse initiale du moteur de la roue gauche
}


//////////////////////////////////////////////

MyRobot::~MyRobot()  // Destructor of MyRobot class / Destructeur de la classe MyRobot
{
    _left_wheel_motor->setVelocity(0.0);  // Set velocity of left wheel motor to zero / Mettre la vitesse du moteur de la roue gauche à zéro
    _right_wheel_motor->setVelocity(0.0);  // Set velocity of right wheel motor to zero / Mettre la vitesse du moteur de la roue droite à zéro
    _my_compass->disable();  // Disable the compass / Désactiver la boussole
    _left_wheel_sensor->disable();  // Disable the left wheel position sensor / Désactiver le capteur de position de la roue gauche
    _right_wheel_sensor->disable();  // Disable the right wheel position sensor / Désactiver le capteur de position de la roue droite
    _gps->disable();  // Disable the GPS / Désactiver le GPS
    _forward_camera->disable();  // Disable the forward camera / Désactiver la caméra avant
}



//////////////////////////////////////////////
void MyRobot::run(){
    
    this->go_to_start();  // Move to the starting position / Aller à la position de départ
    while (step(_time_step) != -1)  // Continue looping until the simulation ends / Continuer à boucler tant que la simulation n'est pas terminée
    {
        const unsigned char* image = _forward_camera->getImage();  // Get image from the forward camera / Obtenir l'image de la caméra avant
        int width = _forward_camera->getWidth();  // Get the image width / Obtenir la largeur de l'image
        int height = _forward_camera->getHeight();  // Get the image height / Obtenir la hauteur de l'image
        
        int center_x = width / 2;  // X-coordinate of the image center / Coordonnée X du centre de l'image
        int center_y = height / 2;  // Y-coordinate of the image center / Coordonnée Y du centre de l'image
    
        int r = _forward_camera->imageGetRed(image, width, center_x, center_y);  // Get the red value at the center pixel / Obtenir la valeur rouge du pixel central
        int g = _forward_camera->imageGetGreen(image, width, center_x, center_y);  // Get the green value at the center pixel / Obtenir la valeur verte du pixel central
        int b = _forward_camera->imageGetBlue(image, width, center_x, center_y);  // Get the blue value at the center pixel / Obtenir la valeur bleue du pixel central
    
        //cout << "🎨 RGB at center - R: " << r << " G: " << g << " B: " << b << endl;  

        
        if( (40 < r && r < 50) && (106 < g && g < 116 ) && ( 106 < b && b < 116) )  // Check for a specific color (red range) / Vérifier une couleur spécifique (plage de rouge)
        {
         world = this->routineRouge();  // Call the routine for the red world / Appeler la routine pour le monde rouge
         cout << " world : " << world << endl;  
        }
        
        if( (160 < r && r < 170) && (247 < g && g < 257 ) && ( 247 < b && b < 257) )  // Check for another specific color (blue range) / Vérifier une autre couleur spécifique (plage de bleu)
        {
        //cout << "routine black blue" << endl;  // Print message indicating black-blue routine / Afficher un message indiquant la routine noir-bleu
        world = 9;  // Set world to 9 / Définir le monde à 9
        cout << " world : " << world << endl;  
        }
        
        if( (57 < r && r < 67) && (140 < g && g < 160 ) && (140 < b && b < 160) )  // Check for a specific color (blue range) / Vérifier une couleur spécifique (plage de bleu)
        {
        //cout << "routine blue" << endl;  // Print message indicating blue routine / Afficher un message indiquant la routine bleue
        world = 3;  // Set world to 3 / Définir le monde à 3
        cout << " world : " << world << endl; 
        }
        
        if( (83 < r && r < 89) && (183 < g && g < 193 ) && (183 < b && b < 193) )  // Check for another specific color (black range) / Vérifier une autre couleur spécifique (plage de noir)
        {
        //cout << "routine black" << endl;  // Print message indicating black routine / Afficher un message indiquant la routine noire
        world = 2;  // Set world to 2 / Définir le monde à 2
        cout << " world : " << world << endl; 
        }
        
        if( (88 < r && r < 94) && (187 < g && g < 197 ) && (187 < b && b < 197) )  // Check for green color range / Vérifier la plage de couleur verte
        {
        world = this->routineVerte();  // Call the green routine / Appeler la routine verte
        cout << " world : " << world << endl; 
        }
        
        if (world > 0 && world <= 10) {  // Check if world is within valid range / Vérifier si le monde est dans la plage valide
              const float (*path1)[2];
              const float (*path2)[2];
              const float (*path3)[2];

              int length1 = 0;
              int length2 = 0;
              int length3 = 0;
              
              switch ( world) {  // Select path based on the world value / Sélectionner le chemin en fonction de la valeur du monde
                  case 1: path1 = world1_path1; length1 = sizeof(world1_path1)/sizeof(world1_path1[0]);
                          path2 = world1_path2; length2 = sizeof(world1_path2)/sizeof(world1_path2[0]);
                          path3 = world1_path3; length3 = sizeof(world1_path3)/sizeof(world1_path3[0]);
                          break;
                  case 2: path1 = world2_path1; length1 = sizeof(world2_path1)/sizeof(world2_path1[0]);
                          path2 = world2_path2; length2 = sizeof(world2_path2)/sizeof(world2_path2[0]);
                          path3 = world2_path3; length3 = sizeof(world2_path3)/sizeof(world2_path3[0]);
                          break;
                  case 3: path1 = world3_path1; length1 = sizeof(world3_path1)/sizeof(world3_path1[0]);
                          path2 = world3_path2; length2 = sizeof(world3_path2)/sizeof(world3_path2[0]);
                          path3 = world3_path3; length3 = sizeof(world3_path3)/sizeof(world3_path3[0]);
                          break;
                  case 4: path1 = world4_path1; length1 = sizeof(world4_path1)/sizeof(world4_path1[0]);
                          path2 = world4_path2; length2 = sizeof(world4_path2)/sizeof(world4_path2[0]);
                          path3 = world4_path3; length3 = sizeof(world4_path3)/sizeof(world4_path3[0]);
                          break;
                  case 5: path1 = world5_path1; length1 = sizeof(world5_path1)/sizeof(world5_path1[0]);
                          path2 = world5_path2; length2 = sizeof(world5_path2)/sizeof(world5_path2[0]);
                          path3 = world5_path3; length3 = sizeof(world5_path3)/sizeof(world5_path3[0]);
                          break;
                  case 6: path1 = world6_path1; length1 = sizeof(world6_path1)/sizeof(world6_path1[0]);
                          path2 = world6_path2; length2 = sizeof(world6_path2)/sizeof(world6_path2[0]);
                          path3 = world6_path3; length3 = sizeof(world6_path3)/sizeof(world6_path3[0]);
                          break;
                  case 7: path1 = world7_path1; length1 = sizeof(world7_path1)/sizeof(world7_path1[0]);
                          path2 = world7_path2; length2 = sizeof(world7_path2)/sizeof(world7_path2[0]);
                          path3 = world7_path3; length3 = sizeof(world7_path3)/sizeof(world7_path3[0]);
                          break;
                  case 8: path1 = world8_path1; length1 = sizeof(world8_path1)/sizeof(world8_path1[0]);
                          path2 = world8_path2; length2 = sizeof(world8_path2)/sizeof(world8_path2[0]);
                          path3 = world8_path3; length3 = sizeof(world8_path3)/sizeof(world8_path3[0]);
                          break;
                  case 9: path1 = world9_path1; length1 = sizeof(world9_path1)/sizeof(world9_path1[0]);
                          path2 = world9_path2; length2 = sizeof(world9_path2)/sizeof(world9_path2[0]);
                          path3 = world9_path3; length3 = sizeof(world9_path3)/sizeof(world9_path3[0]);
                          break;
                  case 10:path1 = world10_path1; length1 = sizeof(world10_path1)/sizeof(world10_path1[0]);
                          path2 = world10_path2; length2 = sizeof(world10_path2)/sizeof(world10_path2[0]);
                          path3 = world10_path3; length3 = sizeof(world10_path3)/sizeof(world10_path3[0]);
                          break;
                  default: cout << "No path defined for this world." << endl; return;  // Handle invalid world case / Gérer le cas d'un monde invalide
              }
          
              follow_path(path1, length1);  // Follow the first path / Suivre le premier chemin
            //   int g = compute_green_percentage();  // Compute green percentage / Calculer le pourcentage de vert

            //   while(g < 0.20){  // Continue rotating until enough green is detected / Continuer à tourner jusqu'à ce qu'un pourcentage suffisant de vert soit détecté
            //       double target_angle = convert_deg_to_rad(10);  // Target rotation angle / Angle de rotation cible
            //       rotate_to_angle(target_angle);  // Rotate to the target angle / Tourner vers l'angle cible
            //       g = compute_green_percentage();  // Update green percentage / Mettre à jour le pourcentage de vert
            //   }
            //   turn_full_circle();  // Perform a full circle turn / Effectuer un tour complet
            //   follow_path(path2,length2);  // Follow the second path, go to second peaple / Suivre le deuxième chemin
            //   while(g < 0.20){  // Repeat the same process for the second path / Répéter le même processus pour le deuxième chemin
            //       double target_angle = convert_deg_to_rad(10);  // Target rotation angle / Angle de rotation cible
            //       rotate_to_angle(target_angle);  // Rotate to the target angle / Tourner vers l'angle cible
            //       g = compute_green_percentage();  // Update green percentage / Mettre à jour le pourcentage de vert
            //   }
            //   turn_full_circle();  // Perform a full circle turn / Effectuer un tour complet
            //   follow_path(path3,length3);  // Follow the third path, come back/ Suivre le troisième chemin

        break;
        }
     break;
    }
}

//////////////////////////////////////////////////////////////
void MyRobot::follow_path_begining(const float path[][2], int length) {
    //cout << "FOLLOWING PRESET PATH NOW" << endl;  
    //NOTE: this function has been rendered obselete by the 001 flagging system.
    
    if (world == 2 || world == 3 || world == 9) {  // If the world is 2, 3, or 9, reset position / Si le monde est 2, 3 ou 9, réinitialiser la position
        _x = _y = _theta = 0.0;  // Reset robot's position and orientation / Réinitialiser la position et l'orientation du robot
    }

    for (int i = 0; i < length; ++i) {  // Loop through each point in the path / Boucler à travers chaque point du chemin
        _x_goal = path[i][0];  // Set the X goal from the path / Définir l'objectif X à partir du chemin
        _y_goal = path[i][1];  // Set the Y goal from the path / Définir l'objectif Y à partir du chemin
        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  // Calculate the goal orientation / Calculer l'orientation de l'objectif

        while (step(_time_step) != -1) {  // Loop while the simulation is running / Boucler tant que la simulation est en cours
            compute_odometry();  // Compute the robot's odometry / Calculer l'odométrie du robot
            //print_odometry();  // Print the robot's odometry / Afficher l'odométrie du robot
            go_route();  // Move towards the goal / Se diriger vers l'objectif

            if (goal_reached()) {  // Check if the goal is reached / Vérifier si l'objectif est atteint
                stop();  // Stop the robot / Arrêter le robot
                break;  // Exit the loop once the goal is reached / Quitter la boucle une fois l'objectif atteint
            }
        }
    }
    
    //cout << "FOLLOWING PRESET PATH END" << endl;  
}

///////////////////////////////////////////////////////////////


void MyRobot::follow_path(const float path[][2], int length) {
    //cout << " FOLLOWING PRESET PATH NOW" << endl; 
     //it s the same logic as follow_path_begining
    for (int i = 0; i < length; ++i) {
        _x_goal = path[i][0];
        _y_goal = path[i][1];
        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

        if (ends_in_001(_x_goal) && ends_in_001(_y_goal)) { //This if statement detects if a flag was put on the coordinate point (001), if yes, then the robot turns in a full circle.
            turn_full_circle();
        }

        while (step(_time_step) != -1) {
            compute_odometry();
            //print_odometry();
            go_route();

            if (goal_reached()) {
                stop();
                break;
            }
        }
    }
    
    //cout << " FOLLOWING PRESET PATH END" << endl;
}

bool MyRobot::ends_in_001(float value) {
    value = fabs(value);

    // Step 1: Round to the nearest thousandth
    value = roundf(value * 1000.0f) / 1000.0f;

    // Step 2: Isolate the thousandths digit
    int thousandths_digit = static_cast<int>(value * 1000) % 10;

    bool result = (thousandths_digit == 1);

    // cout << "[ends_in_001 DEBUG] value (rounded): " << value 
         // << ", thousandths digit: " << thousandths_digit 
         // << ", result: " << (result ? "true" : "false") 
         // << endl;

    return result;
}
////////////////////////////////////////////////////////////////

float MyRobot::compute_green_percentage() {
    const unsigned char* image = _forward_camera->getImage();  // Get the image from the forward camera / Obtenir l'image de la caméra avant
    int width = _forward_camera->getWidth();  // Get the width of the image / Obtenir la largeur de l'image
    int height = _forward_camera->getHeight();  // Get the height of the image / Obtenir la hauteur de l'image

    int green_pixel_count = 0;  // Counter for green pixels / Compteur pour les pixels verts
    int total_pixels = width * height;  // Total number of pixels in the image / Nombre total de pixels dans l'image

    for (int y = 0; y < height; ++y) {  // Loop through the image rows / Boucler à travers les lignes de l'image
        for (int x = 0; x < width; ++x) {  // Loop through the image columns / Boucler à travers les colonnes de l'image
            int r = _forward_camera->imageGetRed(image, width, x, y);  // Get the red value of the pixel / Obtenir la valeur rouge du pixel
            int g = _forward_camera->imageGetGreen(image, width, x, y);  
            int b = _forward_camera->imageGetBlue(image, width, x, y);  

            if (g > 100 && g > r + 30 && g > b + 30) {  // Check if the pixel is mostly green / Vérifier si le pixel est principalement vert
                ++green_pixel_count;  // Increment the green pixel counter / Incrémenter le compteur de pixels verts
            }
        }
    }

    float green_percentage = (green_pixel_count * 100.0f) / total_pixels;  // Calculate the percentage of green pixels / Calculer le pourcentage de pixels verts

    return green_percentage;  
}
///////////////////////////////////////////////

// function to find which world is
int MyRobot::routineVerte() {
    //cout << "routine verte" << endl;  

    double target_angle = convert_deg_to_rad(180);  // Set target angle to 180 degrees / Définir l'angle cible à 180 degrés
    rotate_to_angle(target_angle);  // Rotate the robot to the target angle / Tourner le robot vers l'angle cible

    //first point to find a specific wall
    _x = _y = _theta = 0.0;  // Reset robot position and orientation / Réinitialiser la position et l'orientation du robot
    _x_goal = 2.15;  // Set X goal position / Définir la position cible X
    _y_goal = 0;  
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  // Calculate the goal orientation / Calculer l'orientation de l'objectif
    _theta = convert_deg_to_rad(-180);  // Set robot's orientation / Définir l'orientation du robot
    stop();  

    while (step(_time_step) != -1) {  
        this->compute_odometry(); 
        //this->print_odometry();  
        this->go_route();  

        if (this->goal_reached()) {  
            this->stop();  
            double distanceAvant = _distance_sensor[0]->getValue();  // Get the value from the front distance sensor / Obtenir la valeur du capteur de distance avant
            //cout << "Distance avant : " << distanceAvant << endl; 
             
            //if wall = world 6 or 7
            if (distanceAvant == 0) {  
                //cout << "monde 6 ou 7" << endl; 

                // second important point to find a wall
                _x_goal = -1.75;  
                _y_goal = 0.10;  
                _theta_goal = atan2((_y_goal - _y), (_x_goal - _x)); 
                _theta = convert_deg_to_rad(-180);  
                stop();  

                while (step(_time_step) != -1) {  
                    this->compute_odometry(); 
                    //this->print_odometry();  
                    this->go_route();  
                    //cout << "on a fini le parcours" << endl; 

                    if (this->goal_reached()) {  
                        //cout << "goallll" << endl;  
                        this->stop(); this->stop(); 
                        double front_value = _distance_sensor[0]->getValue(); 
                        //cout << "Distance avant : " << front_value << endl; 
                        
                        // if wall = world 6
                        if (front_value > 10) {  
                            return 6;
                        } else {  
                            return 7;
                        }
                    }
                }
                return -1;  //error

            } else { 
                //important point to say if it s 4 or 5
                _x_goal = 1.85;  
                _y_goal = 1.4;  
                _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));  
                _theta = convert_deg_to_rad(-180);  
                stop();  
                
                while (step(_time_step) != -1) {  
                     // cout << "goallll" << endl;  
                    this->compute_odometry();  // Compute the robot's odometry / Calculer l'odométrie du robot
                    //this->print_odometry(); 
                    this->go_route(); 

                    if (this->goal_reached()) {  
                        this->stop(); 
                        double left_side_value = _distance_sensor[3]->getValue();  // Get the value from the left distance sensor / Obtenir la valeur du capteur de distance gauche
                        //cout << "Distance gauche : " << left_side_value << endl;  
                        
                        //if wall = world 5
                        if (left_side_value > 10) {  
                            return 5;
                        } else {  
                            return 4;
                        }
                    }
                }
                return -1;  
            }
        }
    }
    return -1; 
}

//////////////////////////////////////////////////////////

int MyRobot::routineRouge() {
    //same logic as routineVerte to find wich world is it
    
    //cout << "routine rouge" << endl;

    double target_angle = convert_deg_to_rad(-2);
    rotate_to_angle(target_angle);

    compute_odometry(true);
    reset_odometry(true);

    //important point to find a wall or not
    _x_goal = -2.0;
    _y_goal = 0;
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));

    while (step(_time_step) != -1) {
        compute_odometry();
        //print_odometry();
        go_route();

        if (goal_reached()) {
            stop();
            double distanceAvant = _distance_sensor[0]->getValue();
            //cout << "Distance avant : " << distanceAvant << endl;
            
            //if wall is world 8
            if (distanceAvant == 0) {
                //cout << "world 8" << endl;
                return 8;
            } else {
                _x_goal = -2.0;
                _y_goal = 1.4;
                _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));
                _theta = convert_deg_to_rad(-180);
                stop();

                while (step(_time_step) != -1) {
                    this->compute_odometry();
                    //this->print_odometry();
                    this->go_route();

                    if (this->goal_reached()) {
                        this->stop(); this->stop();
                        _x_goal = -2.4;
                        _y_goal = 0.5;
                        _theta_goal = atan2((_y_goal - _y), (_x_goal - _x));
                        _theta = convert_deg_to_rad(-180);
                        stop();

                        while (step(_time_step) != -1) {
                            this->compute_odometry();
                            //this->print_odometry();
                            this->go_route();

                            if (this->goal_reached()) {
                                this->stop(); this->stop();

                                double front_value        = _distance_sensor[0]->getValue();
                                double front_left_value   = _distance_sensor[1]->getValue();
                                double front_right_value  = _distance_sensor[14]->getValue();

                               

                                if (front_value > 50 && front_left_value > 50 && front_right_value > 50) {
                                    //cout << "world 10" << endl;
                                    return 10;
                                } else {
                                    //cout << "world 1" << endl;
                                    return 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return -1; 
}





//////////////////////////////////////////////

void MyRobot::go_to_start()
{
    step(_time_step); 
    
    // rotate to the wall thank compas
    rotate_to_compass_angle(convert_deg_to_rad(0));  


    //find the wall
    while (step(_time_step) != -1) {  
        double front_value = _distance_sensor[0]->getValue();  // Get the value from the front distance sensor / Obtenir la valeur du capteur de distance avant

        if (front_value > 10) {  // If distance is greater than 10, stop the robot / Si la distance est supérieure à 10, arrêter le robot
            stop();  // Stop the robot / Arrêter le robot
            break;  // Exit the loop / Quitter la boucle
        }

        _left_speed = MAX_SPEED - 2; 
        _right_speed = MAX_SPEED - 2; 
        _left_wheel_motor->setVelocity(_left_speed);  
        _right_wheel_motor->setVelocity(_right_speed); 
    }


    // turn the robot to the corner
        
    while (step(_time_step) != -1) {  
        double left_value = _distance_sensor[3]->getValue();  // Get the value from the left distance sensor / Obtenir la valeur du capteur de distance gauche
        
        
        if (left_value > 500) {  // If the left sensor detects a wall, stop the robot / Si le capteur gauche détecte un mur, arrêter le robot
            stop();  // Stop the robot / Arrêter le robot          
            break;  
        }
        
        _left_speed = SLOW_SPEED;  
        _right_speed = -SLOW_SPEED; 
        _left_wheel_motor->setVelocity(_left_speed); 
        _right_wheel_motor->setVelocity(_right_speed); 
    }
    
    // follow the wall and go to the corner
    while (step(_time_step) != -1) {  
        double front_value = _distance_sensor[0]->getValue(); 
        double left_value  = _distance_sensor[3]->getValue();  // Get the value from the left distance sensor / Obtenir la valeur du capteur de distance gauche 
    
    
        if (left_value > 10) {  // If the robot is too close to the left wall, adjust the speed / Si le robot est trop près du mur gauche, ajuster la vitesse
            _left_speed = MEDIUM_SPEED - 1;  
            _right_speed = MEDIUM_SPEED - 3; 
        } else if (left_value < 400) {  // If the robot is too far from the left wall, adjust the speed / Si le robot est trop loin du mur gauche, ajuster la vitesse
            _left_speed = MEDIUM_SPEED - 3;  
            _right_speed = MEDIUM_SPEED - 1; 
        } else {  // If the robot is at the correct distance from the left wall, maintain maximum speed / Si le robot est à la bonne distance du mur gauche, maintenir la vitesse maximale
            _left_speed = MAX_SPEED - 2;  
            _right_speed = MAX_SPEED - 2; 
        }
        
        if (front_value > 700) {  // If the front distance is greater than 700, stop the robot and finish the process / Si la distance avant est supérieure à 700, arrêter le robot et terminer le processus
            stop();  
            break;  
        }
        
        _left_wheel_motor->setVelocity(_left_speed);  
        _right_wheel_motor->setVelocity(_right_speed);  

}
}

///////////////////////////////////////////////////////////////////////////////


void MyRobot::rotate_to_compass_angle(double target_angle_rad) {
    while (step(_time_step) != -1) {  
        double current_angle = convert_bearing_to_radians();  // Get the current angle from the compass / Obtenir l'angle actuel de la boussole
        double angle_diff = target_angle_rad - current_angle;  // Calculate the difference between the target angle and current angle / Calculer la différence entre l'angle cible et l'angle actuel

        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;  // Normalize angle difference to range [-pi, pi] / Normaliser la différence d'angle dans la plage [-pi, pi]
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;  // Normalize angle difference to range [-pi, pi] / Normaliser la différence d'angle dans la plage [-pi, pi]

        //cout << std::fixed << std::setprecision(2); 
        //cout << " Target: " << convert_rad_to_deg(target_angle_rad)
        //     << "° |  Current: " << convert_rad_to_deg(current_angle)
        //     << "° |  Diff: " << convert_rad_to_deg(angle_diff) << "°" << endl; 

        if (fabs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE)) {  // Check if the robot is within the tolerance / Vérifier si le robot est dans la tolérance
            //cout << " Orientation atteinte." << endl;  
            stop();  
            break;  
        }

        double speed = SLOW_SPEED;  // Set initial speed to slow / Définir la vitesse initiale à lente
        if (fabs(angle_diff) < convert_deg_to_rad(2 * DEGREE_TOLERANCE)) {  // If angle difference is small, reduce speed even further / Si la différence d'angle est petite, réduire encore la vitesse
            speed /= 5.0;  // Reduce speed by a factor of 5 / Réduire la vitesse par un facteur de 5
        }

        if (angle_diff > 0) {  // If the target angle is greater than the current angle, rotate counterclockwise / Si l'angle cible est supérieur à l'angle actuel, tourner dans le sens antihoraire
            _left_speed = -speed;  
            _right_speed = speed; 
        } else {  // If the target angle is smaller than the current angle, rotate clockwise / Si l'angle cible est inférieur à l'angle actuel, tourner dans le sens horaire
            _left_speed = speed;  
            _right_speed = -speed;  
        }

        _left_wheel_motor->setVelocity(_left_speed);  
        _right_wheel_motor->setVelocity(_right_speed);  
    }

    stop(); 
}








//////////////////////////////////////////////

void MyRobot::rotate_to_angle(double target_angle_rad)
{
    while (step(_time_step) != -1) {  
        compute_odometry();  
        double angle_diff = target_angle_rad - _theta;  // Calculate the difference between the target angle and the current angle / Calculer la différence entre l'angle cible et l'angle actuel

        if (angle_diff < -M_PI)  // Normalize the angle difference to range [-pi, pi] / Normaliser la différence d'angle dans la plage [-pi, pi]
            angle_diff += 2 * M_PI;  // Add 2π to angle difference / Ajouter 2π à la différence d'angle
        else if (angle_diff > M_PI)  
            angle_diff -= 2 * M_PI;  // Subtract 2π from angle difference / Soustraire 2π de la différence d'angle

        if (abs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE))  // If the difference is within tolerance, stop rotating / Si la différence est dans la tolérance, arrêter la rotation
            break;

        if (angle_diff > 0) {  // If the target angle is greater, rotate counterclockwise / Si l'angle cible est supérieur, tourner dans le sens antihoraire
            _left_speed = -SLOW_SPEED; 
            _right_speed = SLOW_SPEED;  
        }
        else {  // If the target angle is smaller, rotate clockwise / Si l'angle cible est plus petit, tourner dans le sens horaire
            _left_speed = SLOW_SPEED;  // Set the left wheel speed for clockwise rotation / Définir la vitesse de la roue gauche pour une rotation horaire
            _right_speed = -SLOW_SPEED;  // Set the right wheel speed for clockwise rotation / Définir la vitesse de la roue droite pour une rotation horaire
        }

        if (abs(angle_diff) < convert_deg_to_rad(DEGREE_TOLERANCE * 2)) {  // If the angle difference is smaller than twice the tolerance, reduce speed / Si la différence d'angle est inférieure à deux fois la tolérance, réduire la vitesse
            _left_speed /= 5.0;  // Reduce the left wheel speed / Réduire la vitesse de la roue gauche
            _right_speed /= 5.0;  // Reduce the right wheel speed / Réduire la vitesse de la roue droite
        }

        _left_wheel_motor->setVelocity(_left_speed); 
        _right_wheel_motor->setVelocity(_right_speed);  
    }
}



//////////////////////////////////////////////

void MyRobot::go_route()
{
    if (abs(compute_angle_goal()) > convert_deg_to_rad(DEGREE_TOLERANCE))  // If the goal angle is greater than the tolerance, head towards the goal / Si l'angle cible est supérieur à la tolérance, se diriger vers l'objectif
        head_goal();  // Move towards the goal direction / Se diriger vers l'objectif
    else
        move_forward();  // If the angle is within tolerance, move forward / Si l'angle est dans la tolérance, avancer
}

//////////////////////////////////////////////

void MyRobot::reset_odometry(bool use_compass) {
    compute_odometry(use_compass);  // Update _x, _y, and _theta based on odometry / Mettre à jour _x, _y, et _theta en fonction de l'odométrie
    _sl = encoder_tics_to_meters(_left_wheel_sensor->getValue());  // Convert the left wheel sensor value to meters / Convertir la valeur du capteur de la roue gauche en mètres
    _sr = encoder_tics_to_meters(_right_wheel_sensor->getValue());  // Convert the right wheel sensor value to meters / Convertir la valeur du capteur de la roue droite en mètres

    _x = _y = 0.0;  // Reset the position coordinates to zero / Réinitialiser les coordonnées de position à zéro
    _theta = (use_compass ? convert_bearing_to_radians() : 0.0);  // If using the compass, set the orientation to the compass value, otherwise set to 0 / Si on utilise la boussole, définir l'orientation en fonction de la boussole, sinon définir à 0

    _x_offset = 0.0;  // Reset the X offset to zero / Réinitialiser le décalage X à zéro
    _y_offset = 0.0;  // Reset the Y offset to zero / Réinitialiser le décalage Y à zéro
    _theta_offset = 0.0;  // Reset the theta offset to zero / Réinitialiser le décalage de l'orientation (theta) à zéro
}



//////////////////////////////////////////////
//function view in lab
void MyRobot::compute_odometry(bool use_compass)
{
    float new_sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue());  // Get the left wheel's new encoder value and convert it to meters / Obtenir la nouvelle valeur de l'encodeur de la roue gauche et la convertir en mètres
    float new_sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue());  // Get the right wheel's new encoder value and convert it to meters / Obtenir la nouvelle valeur de l'encodeur de la roue droite et la convertir en mètres

    float diff_sl = new_sl - _sl;  // Calculate the difference in the left wheel's movement / Calculer la différence de mouvement de la roue gauche
    float diff_sr = new_sr - _sr;  // Calculate the difference in the right wheel's movement / Calculer la différence de mouvement de la roue droite

    _sl = new_sl;  // Update the left wheel's position / Mettre à jour la position de la roue gauche
    _sr = new_sr;  // Update the right wheel's position / Mettre à jour la position de la roue droite  

    _x = (_x + ((diff_sr + diff_sl) / 2 * cos(_theta + (diff_sr - diff_sl) / (2 * WHEELS_DISTANCE)))) - _x_offset;  // Update the robot's X position using odometry / Mettre à jour la position X du robot en utilisant l'odométrie
    _y = (_y + ((diff_sr + diff_sl) / 2 * sin(_theta + (diff_sr - diff_sl) / (2 * WHEELS_DISTANCE)))) - _y_offset;  // Update the robot's Y position using odometry / Mettre à jour la position Y du robot en utilisant l'odométrie

    if (use_compass == true)  // If using the compass, update the robot's orientation using the compass / Si on utilise la boussole, mettre à jour l'orientation du robot avec la boussole
        _theta = convert_bearing_to_radians();  // Update orientation with compass value / Mettre à jour l'orientation avec la valeur de la boussole
    else
    {
        _theta = _theta + ((diff_sr - diff_sl) / WHEELS_DISTANCE);  // Update orientation based on wheel movement / Mettre à jour l'orientation en fonction du mouvement des roues

        if (_theta <= -M_PI)  // Ensure theta stays within the range [-pi, pi] / S'assurer que theta reste dans la plage [-pi, pi]
            _theta += 2 * M_PI;  // Adjust angle if below -pi / Ajuster l'angle si inférieur à -pi
        else if (_theta >= M_PI)  
            _theta -= 2 * M_PI;  // Adjust angle if greater than pi / Ajuster l'angle si supérieur à pi
    }

    _theta -= _theta_offset;  // Subtract any offset from the robot's orientation / Soustraire tout décalage de l'orientation du robot
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_radians()
{
    const double *in_vector = _my_compass->getValues();  // Get the values from the compass sensor / Obtenir les valeurs du capteur de boussole
    
    double rad = atan2(in_vector[2], in_vector[0]);  // Convert the compass values to radians / Convertir les valeurs de la boussole en radians

    return rad;  // Return the calculated angle in radians / Retourner l'angle calculé en radians
}
     
//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees()
{
    double rad = convert_bearing_to_radians();  // Convert the bearing (compass) value to radians / Convertir la valeur de la boussole en radians
    double deg = rad * (180.0 / M_PI);  // Convert radians to degrees / Convertir les radians en degrés

    return deg;  // Return the angle in degrees / Retourner l'angle en degrés
}

//////////////////////////////////////////////

double MyRobot::convert_deg_to_rad(double deg)
{    
    double rad = deg * (M_PI / 180.0);  // Convert degrees to radians / Convertir les degrés en radians
    return rad;  // Return the angle in radians / Retourner l'angle en radians
}
//////////////////////////////////////////////

double MyRobot::convert_rad_to_deg(double rad)
{
    double deg = rad * (180.0 / M_PI);  // Convert radians to degrees / Convertir les radians en degrés
    return deg;  // Return the angle in degrees / Retourner l'angle en degrés
}
//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;  // Convert encoder tics to meters based on the encoder's characteristics and the wheel radius / Convertir les tics de l'encodeur en mètres en fonction des caractéristiques de l'encodeur et du rayon de la roue
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y  // Print the current x and y coordinates of the robot / Afficher les coordonnées x et y actuelles du robot
    << " theta:" << _theta  // Print the robot's orientation in radians / Afficher l'orientation du robot en radians
    << " theta degrees:" << _theta * (180.0 / M_PI) << endl;  // Print the robot's orientation in degrees / Afficher l'orientation du robot en degrés
}

//////////////////////////////////////////////

bool MyRobot::goal_reached()
{
    if (abs(_x_goal - _x) < DISTANCE_TOLERANCE && abs(_y_goal - _y) < DISTANCE_TOLERANCE)  // Check if the robot is within the goal's tolerance / Vérifier si le robot est dans la tolérance de l'objectif
        return true;  // Return true if the goal is reached / Retourner vrai si l'objectif est atteint
  
    return false;  // Return false if the goal is not reached / Retourner faux si l'objectif n'est pas atteint
}

//////////////////////////////////////////////

float MyRobot::compute_distance_goal()
{
    float x_target, y_target;
    
    x_target = _x_goal - _x;  // Calculate the difference in x between the goal and current position / Calculer la différence en x entre l'objectif et la position actuelle
    y_target = _y_goal - _y;  // Calculate the difference in y between the goal and current position / Calculer la différence en y entre l'objectif et la position actuelle
    
    double distance = sqrt(pow(x_target, 2) + pow(y_target, 2));  // Calculate the Euclidean distance to the goal / Calculer la distance euclidienne à l'objectif
    return distance;  // Return the distance to the goal / Retourner la distance à l'objectif
}

//////////////////////////////////////////////

float MyRobot::compute_angle_goal()
{
    float x_target, y_target, theta_target;
    
    x_target = _x_goal - _x;  // Calculate the difference in x between the goal and current position / Calculer la différence en x entre l'objectif et la position actuelle
    y_target = _y_goal - _y;  // Calculate the difference in y between the goal and current position / Calculer la différence en y entre l'objectif et la position actuelle
    
    theta_target = atan2(y_target, x_target);  // Calculate the angle to the goal from the current position / Calculer l'angle vers l'objectif depuis la position actuelle
    
    theta_target -= _theta;  // Adjust the target angle by the current orientation / Ajuster l'angle cible avec l'orientation actuelle
    
    if (theta_target < -M_PI)  // Normalize the angle to be within the range [-pi, pi] / Normaliser l'angle pour qu'il soit dans la plage [-pi, pi]
        theta_target += 2 * M_PI;  // Adjust if the angle is less than -pi / Ajuster si l'angle est inférieur à -pi
    else if (theta_target > M_PI)  // Normalize the angle to be within the range [-pi, pi] / Normaliser l'angle pour qu'il soit dans la plage [-pi, pi]
        theta_target -= 2 * M_PI;  // Adjust if the angle is greater than pi / Ajuster si l'angle est supérieur à pi
    
    return theta_target;  // Return the angle to the goal / Retourner l'angle vers l'objectif  
}


//////////////////////////////////////////////

void MyRobot::head_goal()
{    
    float angle_difference = compute_angle_goal();  // Calculate the angle difference to the goal / Calculer la différence d'angle vers l'objectif
   
    if (angle_difference < -convert_deg_to_rad(DEGREE_TOLERANCE))  // If the angle difference is large and negative, rotate counterclockwise / Si la différence d'angle est grande et négative, tourner dans le sens antihoraire
    {
        _left_speed = SLOW_SPEED;  // Set the left wheel speed to slow / Définir la vitesse de la roue gauche à lente
        _right_speed = -SLOW_SPEED;  // Set the right wheel speed to slow in the opposite direction / Définir la vitesse de la roue droite à lente dans la direction opposée
        
        if (angle_difference > -convert_deg_to_rad(DEGREE_TOLERANCE * 2))  // If the angle difference is small, slow down / Si la différence d'angle est petite, ralentir
        {
            _left_speed = SLOW_SPEED / 5.0;  // Reduce the left wheel speed further / Réduire davantage la vitesse de la roue gauche
            _right_speed = (-SLOW_SPEED) / 5.0;  // Reduce the right wheel speed further / Réduire davantage la vitesse de la roue droite
        }
    }
    else if (angle_difference > convert_deg_to_rad(DEGREE_TOLERANCE))  // If the angle difference is large and positive, rotate clockwise / Si la différence d'angle est grande et positive, tourner dans le sens horaire
    {
        _left_speed = -SLOW_SPEED;  // Set the left wheel speed to slow in the opposite direction / Définir la vitesse de la roue gauche à lente dans la direction opposée
        _right_speed = SLOW_SPEED;  // Set the right wheel speed to slow / Définir la vitesse de la roue droite à lente
        
        if (angle_difference < convert_deg_to_rad(DEGREE_TOLERANCE * 2))  // If the angle difference is small, slow down / Si la différence d'angle est petite, ralentir
        {
            _left_speed = (-SLOW_SPEED) / 5.0;  // Reduce the left wheel speed further / Réduire davantage la vitesse de la roue gauche
            _right_speed = SLOW_SPEED / 5.0;  // Reduce the right wheel speed further / Réduire davantage la vitesse de la roue droite
        }
    }
    else  // If the angle difference is within tolerance, stop / Si la différence d'angle est dans la tolérance, arrêter
    {
        _left_speed = 0;  // Set the left wheel speed to zero / Définir la vitesse de la roue gauche à zéro
        _right_speed = 0;  // Set the right wheel speed to zero / Définir la vitesse de la roue droite à zéro
    }
            
    _left_wheel_motor->setVelocity(_left_speed);  // Set the velocity for the left wheel / Définir la vitesse du moteur de la roue gauche
    _right_wheel_motor->setVelocity(_right_speed);  // Set the velocity for the right wheel / Définir la vitesse du moteur de la roue droite
}

//////////////////////////////////////////////

void MyRobot::move_forward()
{
    float distance = compute_distance_goal();  // Calculate the distance to the goal / Calculer la distance vers l'objectif
    if (distance < DISTANCE_TOLERANCE * 10)  // If the goal is close, move at medium speed / Si l'objectif est proche, avancer à vitesse moyenne
    {
        _left_speed = MEDIUM_SPEED;  // Set the left wheel speed to medium / Définir la vitesse de la roue gauche à moyenne
        _right_speed = MEDIUM_SPEED;  // Set the right wheel speed to medium / Définir la vitesse de la roue droite à moyenne
    }
    else  // If the goal is far, move at maximum speed / Si l'objectif est loin, avancer à vitesse maximale
    {
        _left_speed = MAX_SPEED;  // Set the left wheel speed to maximum / Définir la vitesse de la roue gauche à maximale
        _right_speed = MAX_SPEED;  // Set the right wheel speed to maximum / Définir la vitesse de la roue droite à maximale
    }
    _left_wheel_motor->setVelocity(_left_speed);  // Set the velocity for the left wheel / Définir la vitesse du moteur de la roue gauche
    _right_wheel_motor->setVelocity(_right_speed);  // Set the velocity for the right wheel / Définir la vitesse du moteur de la roue droite
}

//////////////////////////////////////////////

void MyRobot::stop()
{
    _left_speed = 0;  // Set the left wheel speed to zero / Définir la vitesse de la roue gauche à zéro
    _right_speed = 0;  // Set the right wheel speed to zero / Définir la vitesse de la roue droite à zéro
 
    _left_wheel_motor->setVelocity(_left_speed);  // Set the velocity for the left wheel to zero / Définir la vitesse du moteur de la roue gauche à zéro
    _right_wheel_motor->setVelocity(_right_speed);  // Set the velocity for the right wheel to zero / Définir la vitesse du moteur de la roue droite à zéro  
}

/////////////////////////////////////
void MyRobot::turn_full_circle() {
    cout << "tu es sauvé // you are safe" << endl;  

    _left_speed = -MAX_SPEED;  // Set the left wheel speed to rotate counterclockwise / Définir la vitesse de la roue gauche pour tourner dans le sens antihoraire
    _right_speed = MAX_SPEED;  // Set the right wheel speed to rotate clockwise / Définir la vitesse de la roue droite pour tourner dans le sens horaire

    _left_wheel_motor->setVelocity(_left_speed);  // Set the velocity for the left wheel / Définir la vitesse du moteur de la roue gauche
    _right_wheel_motor->setVelocity(_right_speed);  // Set the velocity for the right wheel / Définir la vitesse du moteur de la roue droite

    double duration = 3.0;  // Set the duration for the full circle / Définir la durée pour le tour complet
    int steps = (int)(duration * 1000 / _time_step);  // Calculate the number of steps needed to complete the full circle / Calculer le nombre d'étapes nécessaires pour faire le tour complet

    for (int i = 0; i < steps; ++i) {  // Perform the steps for the full circle / Effectuer les étapes pour le tour complet
        step(_time_step);  // Execute one step of the simulation / Exécuter une étape de la simulation
    }

    stop();  // Stop the robot after completing the full circle / Arrêter le robot après avoir fait le tour complet
}

