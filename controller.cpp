#include "controller.h" // WICHTIG: Hier binden wir den Header ein
#include <cmath>

// --- STRUCTS ---
struct VehicleParameters{
    double mass; 
    double Cd;  // drag coeff.
    double A;   // frontal area of the car
    double f;   // rolling resistance
};

struct RoadCondition{
	double friction; 
};

struct WeatherConditions{
	double air_density;
	double wind_speed;
};

const VehicleParameters SEDAN{1705.0, 0.27 , 2.2 ,0.014}; // 2020 BMW 320d- getting values from technical specifications
const VehicleParameters SUV   = {1880.0, 0.34, 2.5, 0.018}; // 2020 BMWX3 xDrive30d
const VehicleParameters SPORT= {1760.0, 0.33, 2.07, 0.012}; // 2020 BMW Z4

const RoadCondition FLAT   = { 1.0};
const RoadCondition ICY    = { 0.5};

const WeatherConditions CALM  = {1.202, 0.0};
const WeatherConditions WINDY = {1.202, 5.0};
const WeatherConditions RAINY = {1.15, 2.0};

class VehicleModel{
    VehicleParameters vehicle;
    RoadCondition road;
    WeatherConditions weather;
    double u0 = 20.0;  
    const double g = 9.81;

    public:
        VehicleModel(const VehicleParameters& v,const RoadCondition& r,const WeatherConditions& w)
          : vehicle(v), road(r), weather(w) {}

		double getTau() const{
		    return vehicle.mass /(weather.air_density * vehicle.Cd * vehicle.A *(u0 + weather.wind_speed));
               
		}
		
		double getK() const {
            return 1.0 /(weather.air_density * vehicle.Cd * vehicle.A *(u0 + weather.wind_speed));
        }

        
		
};

class PIDGainCalculator {
    double Ts, zeta;
public:
    PIDGainCalculator(double sample_time, double damping) : Ts(sample_time), zeta(damping) {}
    void calculate(const VehicleModel& v, double* Kp, double* Ki, double* Kd) {
        double Tau = v.getTau();
		double K = v.getK();
        double omega_n = 4.0 / (Ts * zeta);
        *Kp = ((2.0*zeta*omega_n*Tau) - 1.0)/K;
        *Ki = (Tau*omega_n*omega_n)/K;
        *Kd = (1.0 - K)/Tau;
    }
};

// --- Wrapper Implementierung ---
static VehicleModel* myCar = nullptr;
static PIDGainCalculator* myTuner = nullptr;

extern "C"
void compute_linear_model(int vehicle_id,
                          int road_id,
                          int weather_id,
                          
                          double* K,
                          double* Tau,
                          double* Kp,
                          double* Ki,
                          double* Kd)
{
    VehicleParameters vehicle;
    RoadCondition road;
    WeatherConditions weather;

    switch(vehicle_id){
        case 1: vehicle = SEDAN; break;
        case 2: vehicle = SUV;   break;
        case 3: vehicle = TRUCK; break;
        default: vehicle = SEDAN;
    }

    switch(road_id){
        case 1: road = FLAT; break;
        case 2: road = ICY;  break;
        default: road = FLAT;
    }

    switch(weather_id){
        case 1: weather = CALM;  break;
        case 2: weather = WINDY; break;
        case 3: weather = RAINY; break;
        default: weather = CALM;
    }

    // Model ve PID hesaplamasý yapýlýyor
    VehicleModel car(vehicle, road, weather);
    PIDGainCalculator pid(5.0, 1.0);

    *K   = car.getK();
    *Tau = car.getTau();

    pid.calculate(car, Kp, Ki, Kd);
}

