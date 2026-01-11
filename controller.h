#ifndef CONTROLLER_H
#define CONTROLLER_H

extern "C" void compute_linear_model(int vehicle_id,
                          int road_id,
                          int weather_id,
                          
                          double* K,
                          double* Tau,
                          double* Kp,
                          double* Ki,
                          double* Kd);

#endif
