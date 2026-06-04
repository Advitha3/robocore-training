#include<iostream>

class Sensor{
public:

    virtual double read() const =0;
    virtual const char* name()  const = 0;

    virtual ~Sensor() = default;

    void log() const{
        std::cout <<"{[Telemetry]}" <<name() << "output:" <<read() << std::endl;
    }

};


class IMUSensor : public Sensor{
public:
    double read() const override {return 9.81;}
    const char* name() const override {return "IMUSensor";}

};

class LidarSensor : public Sensor{
public:
    double read() const override {return 3.45;}
    const char* name() const override{return "LidarSensor";}
};

class EncoderSensor : public Sensor{
public:
    double read() const override {return 120.0;}
    const char* name() const override {return "EncoderSensor";}
};

int main(){
    std::cout <<"---Initializing Sensor Array---"<<std::endl;
    //create an array of base class pointers pointing to derive objects on the heap
    Sensor* sensors[3] ={
        new IMUSensor(),
        new LidarSensor(),
        new EncoderSensor()

    };

    std::cout << "\n---Processing Telemetry---"<<std::endl;

    for(int i =0; i<3 ;i++){
    // We just call log(). The vtable figures out exactly which name() and read() to execute!
        sensors[i]->log();
    }

    std::cout <<"\n ---shutting down system---" <<std::endl;
     // clean up the raw heap memory to prevent memory leak

     for(int i =0; i<3;i++){
        delete sensors[i];
     }

     std::cout << "All memory freed successfully." << std::endl;

    return 0;
}
