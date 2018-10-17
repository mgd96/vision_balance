#ifndef _ratethread_H_
#define _ratethread_H_

#include "LIPM2d.h"
#include "pid.h"
#include "global.h"

static FILE *fp;

yarp::os::Port portft0;
yarp::os::Port portft1;
yarp::os::Port portImu;
yarp::os::Port portCamera;

/** Left Leg Device */
yarp::dev::PolyDriver leftLegDevice;
/** Left Leg ControlMode2 Interface */
yarp::dev::IControlMode2 *leftLegIControlMode2;
/** Left Leg PositionControl2 Interface */
yarp::dev::IPositionControl2 *leftLegIPositionControl2; // para control en posicion
/** Left Leg VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *leftLegIVelocityControl2; // para control en velocidad

/** Right Leg Device */
yarp::dev::PolyDriver rightLegDevice;
/** Right Leg ControlMode2 Interface */
yarp::dev::IControlMode2 *rightLegIControlMode2;
/** Right Leg PositionControl2 Interface */
yarp::dev::IPositionControl2 *rightLegIPositionControl2; // para control en posicion
/** Right Leg VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *rightLegIVelocityControl2; // para control en velocidad

/** Trunk Device */
yarp::dev::PolyDriver trunkDevice;
/** Trunk ControlMode2 Interface */
yarp::dev::IControlMode2 *trunkIControlMode2;
/** Trunk PositionControl2 Interface */
yarp::dev::IPositionControl2 *trunkIPositionControl2; // para control en posicion
/** Trunk VelocityControl2 Interface */
yarp::dev::IVelocityControl2 *trunkIVelocityControl2; // para control en velocidad

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {    //Conversion to [ms]

        n = 1;
        r = 1;
        m = 0;
        p = 0;

        rightLegIPositionControl2->positionMove(4, 0); // position in degrees
        leftLegIPositionControl2->positionMove(4, 0);

    }

    void run()
    {

        // Test escalon con rampa // generacion del ANG_ref para test CAM sensor

            if (n < 300){ang_ref = 0.0;}
            if(n>=300 && n<=330){
                //ang_ref = -((5/30)*n-50);
                ang_ref = 1*n;
                ang_ref = ang_ref/30;
                ang_ref = ang_ref-10;
            } // variar desde 1 a 10 grados
            //else if(300>=n<=330){ang_ref = -5;} // variar desde 1 a 10 grados
            if (n > 330){ang_ref = ang_ref;}

            getInitialTime();

            if (n>50 && n<=250){
                offsetIMU();            }

            if (n>300){
                readSensors(); // lectura de la CAM e IMU
                ControlJoints(); // Calculo y generacion de la actuacion en funcion del ANG_ref
                printData();
            }

            if (n>=400){
                n=299;
                m = m + 1;
            }

            cout << endl << "Press ENTER to exit..." << endl;
            cout << "*******************************" << endl << endl;
            saveInFileCsv();
            n++;
            r++;
            cout << n << endl << endl;
            getCurrentTime();

    }

    void getInitialTime()
    {
        if (n==1){init_time = Time::now();}
        init_loop = Time::now();
        it_time = init_loop - it_prev;
        it_prev = init_loop;
    }

    void getCurrentTime()
    {
        act_time = Time::now() - init_time;
        act_loop = Time::now() - init_loop;
    }

    void offsetIMU()    {

        readSensors(); // lectura de la CAM e IMU
        offXimu += + angImu_x;
        offYimu += + angImu_y;
        offZimu += + angImu_z;
        offXcam += + angCam_x;
        offYcam += + angCam_y;
        offZcam += + angCam_z;

        if(n==250){
            offXimu = offXimu/200;
            offYimu = offYimu/200;
            offZimu = offZimu/200;
            offXcam = offXcam/200;
            offYcam = offYcam/200;
            offZcam = offZcam/200;
        }

    }

    void readSensors(){

        Bottle ch0;
        Bottle ch1;
        Bottle imu;
        Bottle camera;

  /*      portft0.read(ch0); // lectura del sensor JR3 ch0
        portft1.read(ch1); // lectura del sensor JR3 ch1
 */       portImu.read(imu); // lectura del sensor IMU
        portCamera.read(camera); // lectura del sensor Camera

        //--- CAMERA-Sensor
        angCam_x = camera.get(0).asDouble();
        angCam_y = camera.get(1).asDouble();
        angCam_z = camera.get(2).asDouble();

/*        //--- FT-Sensor
        _fx0 = ch0.get(0).asDouble();
        _fy0 = ch0.get(1).asDouble();
        _fz0 = ch0.get(2).asDouble();
        _mx0 = ch0.get(3).asDouble();
        _my0 = ch0.get(4).asDouble();
        _mz0 = ch0.get(5).asDouble();

        _fx1 = ch1.get(0).asDouble();
        _fy1 = ch1.get(1).asDouble();
        _fz1 = ch1.get(2).asDouble();
        _mx1 = ch1.get(3).asDouble();
        _my1 = ch1.get(4).asDouble();
        _mz1 = ch1.get(5).asDouble();*/

        //--- Inertial-Sensor
        angImu_x = imu.get(0).asDouble(); // Angulo en X [deg]
        angImu_y = imu.get(1).asDouble(); // Angulo en Y [deg]
        angImu_z = imu.get(2).asDouble(); // Angulo en Z [deg]
/*        acc_x = imu.get(3).asDouble(); //Linear acceleration in X [m/s^2]
        x_sensor.push_front(acc_x);
        x_sensor.pop_back();
        acc_y = imu.get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        y_sensor.push_front(acc_y);
        y_sensor.pop_back();
        acc_z = imu.get(5).asDouble(); //Linear acceleration in Z [m/s^2]
        z_sensor.push_front(acc_z);
        z_sensor.pop_back();
        spd_x=imu.get(6).asDouble(); // Velocidad angular en X [deg/s]
        spd_y=imu.get(7).asDouble(); // Velocidad angular en Y [deg/s]
        spd_z=imu.get(8).asDouble(); // Velocidad angular en Z [deg/s]
        //mag_x=imu.get(9).asDouble(); // Campo magnetico en X
        //mag_y=imu.get(10).asDouble(); // Campo magnetico en Y
        //mag_z=imu.get(11).asDouble(); // Campo magnetico en Z

        //LOW-PASS FILTER
        ddx = 0.0;
        ddy = 0.0;
        ddz = 0.0;
        for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
            ddx = ddx + *it;
        for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
            ddy = ddy + *it;
        for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
            ddz = ddz + *it;
        ddx = ddx / samples;
        ddy = ddy / samples;
        ddz = ddz / samples;

        //CONVERSION FROM IMU SENSOR COORDINATES TO ROBOT COORDINATES
         ddx_robot = ddx;
         ddy_robot = -ddy;
         ddz_robot = ddz;*/

    }

    void ControlJoints(){

        angImu_x = angImu_x - offXimu;
        angImu_y = angImu_y - offYimu;
        angImu_z = angImu_z - offZimu;

        angCam_x = angCam_x - offXcam;
        angCam_y = angCam_y - offYcam;
        angCam_z = angCam_z - offZcam;

        ang_ref2 = ang_ref + m;

   /*         //-- ANKLE STRATEGY
        rightLegIPositionControl2->positionMove(4, -ang_ref2); // position in degrees
        leftLegIPositionControl2->positionMove(4, -ang_ref2);*/

            //-- TRUNK STRATEGY
        trunkIPositionControl2->positionMove(0, -ang_ref2); // position in degrees

}

    void printData()
    {
 /*       cout << endl << "La interacion n es:" << n <<endl;
        cout << endl << "La interacion m es:" << m <<endl;
        cout << endl << "La interacion p es:" << p <<endl;*/
        cout << endl << "El angulo ref es: " << ang_ref2 << endl;

        cout << endl << "El angulo imu X es: " << angImu_x << endl;
        cout << endl << "El angulo imu Y es: " << angImu_y << endl;
        cout << endl << "El angulo imu Z es: " << angImu_z << endl;

        cout << endl << "El ang camera X es: " << angCam_x << endl;
        cout << endl << "El ang camera Y es: " << angCam_y << endl;
        cout << endl << "El ang camera Z es: " << angCam_z << endl;

    }

    void saveInFileCsv()
    {
        if(n==1){
            fprintf(fp,"Time,ang_ref,ang_x_imu,ang_y_imu,ang_z_imu,ang_x_cam,ang_y_cam,ang_z_cam,iter");
        }

        fprintf(fp,"\n%.2f", act_time);
        fprintf(fp,",%.10f", ang_ref2);
        fprintf(fp,",%.10f", angImu_x);
        fprintf(fp,",%.10f", angImu_y);
        fprintf(fp,",%.10f", angImu_z);
        fprintf(fp,",%.10f", angCam_x);
        fprintf(fp,",%.10f", angCam_y);
        fprintf(fp,",%.10f", angCam_z);
        fprintf(fp,",%i", r);

    }

private:
    int m, n, p, r;
    double _num; // Variable para jugar con larefs iteraciones (500 iteraciones -> 15 segundos)
    float e; // distance [m] between ground and sensor center

    LIPM2d _eval_Ctrl;

    //-- CAMERA variables
    double angCam_x,angCam_y,angCam_z;
    double offXcam, offYcam, offZcam; // eliminar el Offset de la CAMARA

    //-- IMU variables
    double acc_x, acc_y, acc_z, angImu_x, angImu_y, angImu_z, spd_x, spd_y, spd_z, mag_x, mag_y, mag_z; // IMU inputs
    double offXimu, offYimu, offZimu; // eliminar el Offset del sensor IMU
    //-- IMU LOW-FILTER variables & CONVERTION
    double ddx, ddy, ddz, ddx_robot, ddy_robot, ddz_robot; // additional acc variables

    double init_time, act_time, init_loop, act_loop, it_time, it_prev; // variables para los tiempos
    deque<double> x_sensor, y_sensor, z_sensor;

    //-- FT variables
    float _fx0, _fy0, _fz0, _mx0, _my0, _mz0; // F-T from sensor 0 [Fuerza en N y Pares en Nm*10]
    float _fx1, _fy1, _fz1, _mx1, _my1, _mz1; // F-T from sensor 1 [Fuerza en N y Pares en Nm*10]
    //-- FT LOW-FILTER variables
    float offs_x_j, offs_x_l, offs_x_l2; // zmp offset in initial time.
    float offs_y;
    float sum_j, sum_l, sum_l2;

    //-- ZMP variables
    float _xzmp0_ft, _yzmp0_ft; // ZMP sensor 0 (derecho)
    float _xzmp1_ft, _yzmp1_ft; // ZMP sensor 1 (izquierdo)
    float _xzmp01_ft, _yzmp01_ft; // ZMP robot (zmp_0 + zmp_0) metros
    float Xzmp_ft, Yzmp_ft; // Global ZMP-FT despues de filtrar
    double Xzmp_imu, Yzmp_imu, Xzmp_total; // Global ZMP-IMU despues de filtrar
    //, Xzmp_b, Yzmp_b, Xzmp_c, Xzmp_off; // x, y, z en [cm]

    //-- CONTROL/MOVEMENT variables
    float X, ref, X2, ref2, y2, _angle_ref, _angle_ref_1, _angle_ref_a, _angle_ref_b, _angle_ref_c, _angle_ref_d, ka;
    double ang_ref, ang_ref2;
    double _angulo, _angulo2, seno_x, seno_z, coseno_z, radianes; //Angulo para compensar las componentes de la aceleración en el modelo cart-table,
                              //Angulo2 para invertir los angulos negativos
    //-- SET & PID variables
    PID *pidcontroller_ankle, *pidcontroller_hip;
    std::string plane;
    double actual_value, setpoint, pid_output_ankle, pid_output_hip, initial_encoder;
    double capture_point, lin_vel, w;

    BufferedPort<Bottle> *readPort;
    IVelocityControl *velTrunk, *velRightLeg, *velLeftLeg;

};

#endif

