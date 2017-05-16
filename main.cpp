
// Final_Test.cpp : Defines the entry point for the console application.
//



//#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <assert.h>
#include <random>
#include <fstream>
#include <numeric>
#include <time.h>
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <limits>
#include <algorithm>

#include "LY_NN.h"

using namespace std;
neural_network NN;
#define PI 3.1415
#define LYRAND (double)rand()/RAND_MAX

int boundary_x_low = 0;
int boundary_y_low = 0;
int boundary_x_high = 100;
int boundary_y_high = 100;


////////////////////////////////////////////////////////////////////////////
///////////////////////////   Policies  ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

class Policies {
public:
    //MR_1:  initialize a population of policies - associated with a fitness
    //fitness //MR_2::  distance for the entire policy, minimal
    //
    double fitness = 0;
    void init_policy(int num_weights); //initialize one policy
    vector<double> weights;
    
};

struct less_than_key
{
    inline bool operator() (const Policies& struct1, const Policies& struct2)
    {
        return (struct1.fitness < struct2.fitness);
    }
};

struct greater_than_key
{
    inline bool operator() (const Policies& struct1, const Policies& struct2)
    {
        return (struct1.fitness > struct2.fitness);
    }
};


void Policies::init_policy(int num_weights) {
    for (int p = 0; p < num_weights; p++) {
        //cout << "Order " << p << endl;
        weights.push_back(0);
        //weights.push_back(0);
    }
    
    
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////   B   O   A   T   /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
class boat {
public:
    double boat_x;
    double boat_y;
    double goal_x1;
    double goal_y1;
    double goal_x2;
    double goal_y2;
    double start_boat_x;
    double start_boat_y;
    double starting_theta;
    double starting_w;
    double beta;
    double v = 3; //velocity //set//
    double w = 0; //angular velocity
    double dt = 0.2; //time step //set//
    double theta; //radians
    double T = 5.0; //set//
    double u;
    void Init();
    double Simulation(ofstream& fout, bool tofile, bool NOISY);
    void find_beta(); //thanks Bryant
    double N(double sigma, double mu);
    double noise(double val, double variance);
    
    //Evolutionary EA;
};

void boat::Init() { //pass in NN and EA
    /// STARTING POSITION OF BOAT ///
    //start_boat_x = rand() % boundary_x_high;
    //start_boat_y = rand() % boundary_y_high;
    start_boat_x = 60;
    start_boat_y = 20;
    boat_x = start_boat_x;
    boat_y = start_boat_y;
    beta = 0;
    u = 0;
    /// ORIENTATION OF AGENT ///
    double theta_deg = rand()%360; ///random degree orientation
    starting_theta = theta_deg * PI / 180; /// converts degrees to radians
    //starting_theta = 0;
    theta = starting_theta;
    
    /// ANGULAR SPEED OF AGENT ///
    starting_w = 0;
    //starting_w = 0;
    w = starting_w;
    
    /// GOAL POSITION ///
    //goal_x1 = rand() % boundary_x_high;
    //goal_y1 = rand() % (boundary_y_high-2);
    //goal_x2 = goal_x1;
    //goal_y2 = goal_y1 - 2;
    goal_x1 = 10; //testing
    goal_y1 = 10; //testing
    goal_x2 = 10; //testing
    goal_y2 = 30; //testing
    
}

double boat::Simulation(ofstream &fout, bool PUT_TO_FILE, bool NOISY) {
    //pass in weights
    double y;
    double m;
    double b;
    double boat_x1;
    double boat_y1;
    double time;
    double distance;
    double distance_x;
    double distance_y;
    double stray;
    double min_distance;
    double start_goal_distance = 0;
    double sum_distance;
    double fitness = 0;
    
    /// INITIALIZE STARTING POSITIONS //
    boat_x = start_boat_x;
    boat_y = start_boat_y;
    w = starting_w;
    //cout << boat_x << "," << boat_y << endl;
    theta = starting_theta;
    distance = 0;
    sum_distance = 0;
    /// CALCULATE Minimum Max-DISTANCE TO GOAL ///
    distance_x = pow(goal_x1 - boat_x, 2);
    distance_y = pow(goal_y1 - boat_y, 2);
    //cout << "d_x   " << distance_x << '\t' << "d_y   " << distance_y << endl;
    start_goal_distance = sqrt(distance_x + distance_y);
    min_distance = start_goal_distance;
    
    
    //cout << "before time step loop" << endl;
    for (int i = 0; i < 1000; i++) {
        
        /// CALCULATE THE STRAY FROM THE GOAL ///
        find_beta();
        stray = beta - theta;
        while (stray <= -PI) {
            stray += 2 * PI;
        }
        while (stray > PI) {
            stray = 2 * PI - stray;
        }
        
        
        //cout << "inside time step loop" << endl;
        // Get input vector for NN - x,y,w,theta
        vector<double> state;
        //state.push_back(cos(stray));
        //state.push_back(sin(stray));
        state.push_back(stray);
        //state.push_back(w);
        //state.push_back(boat_x);
        //state.push_back(boat_y);
        //state.push_back(theta);
        NN.set_vector_input(state);
        //Give to NN
        
        
        
        //cout << boat_x << ',' << boat_y << endl;
        //cout << w << endl;
        /// GET VALUE OF U FROM NN
        NN.execute();
        //cout << "poop" << endl;
        if (NOISY == false){
            u = NN.get_output(0)* PI / 180;
        }
        else if (NOISY==true){
            u = NN.get_output(0)* PI / 180 + noise(u, 0.1);
        }
        //cout << "u" << u << endl;
        
        //cout << "S:U\t" << stray << "\t" << u << endl;
        
        
        /// CALCULATE X,Y,THETA,W ///
        
        if (NOISY==false){
            boat_x1 = boat_x + v*cos(theta)*dt;
            boat_y1 = boat_y + v*sin(theta)*dt;
            theta = theta + w*dt;
            if (theta > (1 * PI)) {
                theta = theta - 2 * PI;
            }
            else if (theta < (-1 * PI)) {
                theta = theta + 2 * PI;
            }
            w = w + ((u - w)*dt) / T;
            
        }
        else if (NOISY==true){
            boat_x1 = boat_x + v*cos(theta)*dt+noise(boat_x, 0.5);
            boat_y1 = boat_y + v*sin(theta)*dt+noise(boat_y, 0.5);
            theta = theta + w*dt+noise(theta,0.1);
            if (theta > (1 * PI)) {
                theta = theta - 2 * PI;
            }
            else if (theta < (-1 * PI)) {
                theta = theta + 2 * PI;
            }
            w = w + ((u - w)*dt) / T + noise(w,0.1);
            
        }
        
        
        /// CALCULATIONS FOR INTERCEPT ///
        m = (boat_y1 - boat_y) / (boat_x1 - boat_x); ///slope
        b = boat_y1 - m*boat_x1; /// y intercept
        y = m*goal_x1 + b; ///equation of a line
        
        if (boat_x1 < boat_x) {		//If x1 is to the left of x2
            if (boat_x1 <= goal_x1 && boat_x >= goal_x2) {	//If they are on either side of the goal
                if (y >= goal_y1 && y <= goal_y2) {
                    sum_distance = distance - 5 * (1000 - i);
                    //cout << "FOUND GOAL\t";
                    if(PUT_TO_FILE){
                        fout << boat_x << ',' << boat_y << ',' << theta << ',' << w << ',' << stray << ',' << u << ',' << sum_distance << ',' << i << endl;}
                    break;
                }
            }
        }
        else {		//If x2 is to the left of x1
            if (boat_x <= goal_x1 && boat_x1 >= goal_x2) {	//If they are on either side of the goal
                if (y >= goal_y1 && y <= goal_y2) {
                    sum_distance = distance - 5 * (1000 - i);
                    //cout << "FOUND GOAL\t";
                    if(PUT_TO_FILE){
                        fout << boat_x << ',' << boat_y << ',' << theta << ',' << w << ',' << stray << ',' << u << ',' << sum_distance << ',' << i << endl;}
                    break;
                }
            }
        }
        
        //cout << "boat not close to goal" << endl;
        
        /// UPDATE NEW X,Y, VALUES ///
        // no noise here since it noise was calculated in boat_x1 and boat_y1
       boat_x = boat_x1; ///setting the new x value
       boat_y = boat_y1; ///setting the new y value
        
        
        
        
        if(PUT_TO_FILE){
            fout << boat_x << ',' << boat_y << ',' << theta << ',' << w << ',' << stray << ',' << u << ',' << sum_distance << ',' << i << endl;}
        //cout << boat_x << ',' << boat_y << ',' << theta << ',' << w << endl;
        
        /// CALCULATE DISTANCE TO GOAL ///
        distance_x = pow(goal_x1 - boat_x, 2);
        distance_y = pow(goal_y1 - boat_y, 2);
        //cout << "d_x   " << distance_x << '\t' << "d_y   " << distance_y << endl;
        distance = sqrt(distance_x + distance_y);
        sum_distance = sum_distance + fabs(stray);
        
        /// CONDITIONS TO QUIT THE LOOP ////
        if (boat_x < boundary_x_low || boat_x > boundary_x_high || boat_y < boundary_y_low || boat_y > boundary_y_high) {
            sum_distance += distance + 2.5 * (1000 - i);
            //cout << "OUTSIDE\t";
            if(PUT_TO_FILE){
                fout << boat_x << ',' << boat_y << ',' << theta << ',' << w << ',' << stray << ',' << u << ',' << sum_distance << ',' << i << endl;}
            break;
        }
        assert(boat_x > boundary_x_low);
        assert(boat_y > boundary_y_low);
        assert(boat_x < boundary_x_high);
        assert(boat_y < boundary_y_high);
        //cout << "boat within boundary" << endl;
        
        
    } //for loop
    
    ////////// EXITING COORDINATES ////////
    //cout << s << "\t" << boat_x << ',' << boat_y << endl;
    
    /// CALCULATE THE FITNESS - uses distance and time // MR_4 //
    fitness = sum_distance; //overall distance it took to get to the goal
    //cout << fitness << endl;
    //fout << "fitness" << "," << fitness << endl;
    return fitness;
    //population[s].fitness = fabs(fitness);
}

void boat::find_beta() {
    beta = atan((boat_y - ((goal_y1 + goal_y2) / 2)) / (boat_x - goal_x1));
    if (boat_x > goal_x1) {
        beta += PI;
    }
    else if (boat_x<goal_x1 && boat_y >((goal_y1 + goal_y2) / 2)) {
        beta += 2 * PI;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////   EVOLUTIONARY ALGORITHM  ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
vector<Policies> EA_Replicate(vector<Policies> population, int num_weights) {
    //Take vector of policies and double it
    // Mutate the doubled policies slightly
    int R;
    int O;
    int num = population.size();
    
    int n = num_weights; //number of mutations
    
    vector<Policies> Gen;
    Gen = population; //Copies the old population
    
    for (int i = 0; i < num; i++) {
        R = rand() % (population.size());
        Gen.push_back(population.at(R));
        
        for (int x = 0; x < n; x++) {
            O = rand() % num_weights;
            
            if(rand()%4==0){
                Gen.back().weights.at(O) = Gen.back().weights.at(O) + LYRAND - LYRAND ;
            }
            else{
                Gen.back().weights.at(O) = Gen.back().weights.at(O) + 0.25*LYRAND - 0.25*LYRAND ;
            }
            //if (population.at(R).weights.at(O) > 1) {
            //    population.at(R).weights.at(O) = 1;
            //}
            //else if (population.at(R).weights.at(O) < -1) {
            //    population.at(R).weights.at(O) = -1; //keeps weights within 1 and -1
            //}
            //cout << "Poptry\t" << population.at(R).weights.at(O) << endl;
        }
        
        
        //assert(Gen[R].weights != Pop[R].weights); //LR_4
        
    }
    return Gen;
}


vector<Policies> EA_Downselect(vector<Policies> population) { //Binary Tournament - Take
    // take the fitness of one policy and compare it to another fitness of another policy at random.
    // whichever one has the lower fitness gets put into a new vector until size(population/2)
    vector<Policies> Pop_new;
    int num = population.size();
    //cout << num << ") ";
    
    int best;
    best = -1;
    double bestval = 999999999999;
    for(int i=0; i<population.size(); i++){
        if(population.at(i).fitness < bestval){
            best = i;
            bestval = population.at(i).fitness;
        }
    }
    assert(best!=-1);
    
    Pop_new.push_back(population.at(best));
    
    for (int i = 1; i < num / 2; i++) {
        int R;
        int S;
        R = rand() % num;
        S = rand() % (num);
        //cout << "R\t" << R << endl;
        while (R == S) { //to make sure R and S aren't the same
            S = rand() % num;
        }
        //cout << "S\t" << S << endl;
        if (population.at(R).fitness < population.at(S).fitness) {
            /// "R WINS"
            Pop_new.push_back(population.at(R));
            //cout << population.at(R).fitness << endl;
        }
        
        else {
            /// "S WINS"
            Pop_new.push_back(population.at(S));
            //cout << population.at(S).fitness << endl;
        }
        //cout << Pop_new.size() << " ";
    }
    //cout << endl;
    assert(Pop_new.size() == population.size() / 2); //MR_4
    //return that new vector
    
    return Pop_new;
}


////// NOISE //////////

double boat::N(double sigma, double mu){ //normal distribution
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;
    
    static double z0, z1;
    static bool generate;
    generate = !generate;
    
    if (!generate)
        return z1 * sigma + mu;
    
    double u1, u2;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    } while (u1 <= epsilon);
    
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

double boat::noise(double val, double variance){
    double changed_val = val + N(0,variance);
    return changed_val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////


int main()
{
    
    int MAX_GENERATIONS = 1500;
    int pop_size = 100;
    srand(time(NULL));
    
    bool NOISY = false; //change this if you want noise //false means no noise
    
    //Evolutionary EA;
    int num_weights = 0;
    
    /// SET UP NEURAL NETWORK ///
    
    NN.setup(1, 3, 1); ///3 input, 5 hidden, 1 output (Bias units hidden from user)
    
    //for stray
    NN.set_in_min_max(-3, 3);
    //NN.set_in_min_max(-4, 4);
    
    /// FOR X-VALUES
    //NN.set_in_min_max(0.0, boundary_x_high); /// limits of input for normalization
    /// FOR Y-VALUES
    //NN.set_in_min_max(0.0, boundary_y_high); /// limits of input for normalization
    /// FOR THETA
    //NN.set_in_min_max(0.0, 6.28);
    /// FOR U
    NN.set_out_min_max(-15.0, 15.0); /// limits of outputs for normalization
    
    //NN.set_vector_input(vi); /// vector of inputs
    num_weights = NN.get_number_of_weights();
    
    /// DEFINE STARTING POSITION AND GOAL POSITION ///
    boat B;
    B.Init();
    
    /// INITIALIZE POLICIES ///
    vector<Policies> population;
    for (int p = 0; p < pop_size; p++) {
        Policies A;
        A.init_policy(num_weights);
        population.push_back(A);
    }
    assert(population.size() == pop_size);
    
    ////////// START SIMULATION ///////////////
    ofstream fout; //Movements
    /// MR_3 ///
    fout.open("Movement.csv", ofstream::out | ofstream::trunc);
    fout << "Coordinates of Boat for each time step" << "\n";
    
    bool PUT_TO_FILE=false;
    
    for (int g = 0; g < MAX_GENERATIONS; g++) {
        //cout << population.size() << endl;
        fout << "\nGEN" << g << "  ";
        cout << "GEN" << g << endl;
        
        for (int s = 0; s < population.size(); s++) {
            fout <<"," <<"Sim" << s << ",";
            //cout << population.size() << endl;
            NN.set_weights(population.at(s).weights, true);
            
            if(g%50==0 && s==0){PUT_TO_FILE=true;}
            if(g==MAX_GENERATIONS-1 && s==0){PUT_TO_FILE=true;}
            //if(g==0 && s==0){PUT_TO_FILE=true;}
            //if(g==0 && s==0){PUT_TO_FILE=true;}
            population.at(s).fitness = B.Simulation(fout,PUT_TO_FILE,NOISY);
            PUT_TO_FILE=false;
            //cout << num_weights << endl;
            
            // UPDATE EA WITH FITNESS
            
            
            
        }
        /// EA - DOWNSELECT WITH GIVEN FITNESS
        sort(population.begin(),population.end(),less_than_key());
        population = EA_Downselect(population);
        /// EA - MUTATE and repopulate WEIGHTS
        population = EA_Replicate(population, num_weights);
        
        
        //sort(population.begin(),population.end(),less_than_key());
        //if(g%10==0){
        //for(int j=0; j<population.size(); j++){
        //    cout << population.at(j).fitness << "\t";
        //}
        //cout << endl;
        //}
        
    }
    fout.close();
    cout << "pop size" << population.size() << endl;
    //////// MR_2 ///////////
    //assert(B.boat_y <= B.goal_y2 && B.boat_y >= B.goal_y1 && B.boat_x <= (B.goal_x2 + .05*B.goal_x2) && B.boat_x >= (B.goal_x2 - .05*B.goal_x2));
    //cout << "Boat passed through goal" << endl;
    
    //sort(population.begin(), population.end(), less_than_key());
    
    //int input;
    //cin >> input;
    return 0;
}