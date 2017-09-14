#ifndef PRINTOPENSIMINFORMATION_H
    #define PRINTOPENSIMINFORMATION_H

#include <OpenSim/OpenSim.h>
#include <iostream>
#include <array>
#include <vector>
#include <fstream>
#include <string>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


namespace OpenSim {
    class PrintInformation {
    public:
        PrintInformation(std::ostream& stream) : stream(stream), endl('\n'){}
        PrintInformation() : stream(cout), endl('\n') {}

        void printVector(const SimTK::Vector &vec);
        void printVec3(const SimTK::Vec3& vec);
        void printInertia(const SimTK::Inertia& inert);
        void printSpatialMat(const SimTK::SpatialMat &spatialMat, bool matlab = true);

    protected:
        std::ostream& stream;
        const char endl;
    };


    class PrintToFile  {
    public:
        PrintToFile(const char* fileName) : fileName(fileName),
            file(fileName, std::ios::out | std::ios::trunc)
        {
        }
        void PrintDataToFile(std::vector<string> nameData, std::vector<double> data);
        void PrintDataToFile(string* nameData, double* data, int size);

    protected:

        const char* fileName;
        ofstream file;


    };

    class PrintModelInformation : public PrintInformation
    {
    public:
        PrintModelInformation(const Model& osimModel, std::ostream& stream) :
        PrintInformation(stream),aModel(osimModel)
        {

        }

        void printCoordinateNames();

    private:
        const Model& aModel;

    };

    class PrintJointInformation : public PrintInformation
    {
    public:
        PrintJointInformation(const JointSet& joints, std::ostream& stream) :
            PrintInformation(stream), joints(joints) {
            joints.getNames(arrayOfJointNames);
        }

        void printJointNames();
        void printJointParentLocation(int index);
        void printJointChildLocation(int index);
        void printJointParentOrientation(int index);
        void printAllJointParentLocations();
        void printAllJointParentOrientation();

    private:
        JointSet joints;
        Array<std::string> arrayOfJointNames;
    };

    class PrintForceToFile : public PrintToFile {
    public:
        PrintForceToFile(const char* fileName, string forceName, Model* osimModel)
            : PrintToFile(fileName), forceName(forceName), osimModel(osimModel),
                forceSet(osimModel->getForceSet()) {

            forceReporter = new ForceReporter(osimModel);
            osimModel->addAnalysis(forceReporter);


        }

        void getForces();
        void getForceNames();
        void printForces();

    private:
        string forceName;
        Model* osimModel;
        Array<Array<double> > rData;
        ForceReporter* forceReporter;
        ForceSet forceSet;
        Array<string> forceNames;

    };

    class AngularAccelerationReporter : public PeriodicEventReporter {
    public:
        AngularAccelerationReporter(const MultibodySystem& system, const MobilizedBody& mobod,
                     const string moBodyName, Real reportInerval)
            : PeriodicEventReporter(reportInerval), system(system), mobod(mobod), moBodyName(moBodyName) {}
        void handleEvent(const State& state) const {
            system.realize(state, Stage::Acceleration);
            Vec3 angAcc = mobod.getBodyAngularAcceleration(state);
            cout << "Angular Acceleration of " << moBodyName << " = { x = " << angAcc[0] << ", y = "
                 << angAcc[1] << ", z = " << angAcc[2] << " }" << endl;
        }

    private:
        const MultibodySystem& system;
        const MobilizedBody& mobod;
        const string moBodyName;
    };

    class PositionReporter : public PeriodicEventReporter {
    public:
        PositionReporter(const MultibodySystem &system, const MobilizedBody &mobod,
                         const string moBodyName, Real reportInterval)
            :   PeriodicEventReporter(reportInterval), system(system), mobod(mobod), moBodyName(moBodyName) {}
        void handleEvent(const State &state) const {
            system.realize(state, Stage::Position);
            Vec3 pos = mobod.getBodyOriginLocation(state);
            cout << "Position of "<< moBodyName << "= { x = " << pos[0] << ", y = " << pos[1]
                 << ", z = "<< pos[2] << " }" << endl;
        }
    private:
        const MultibodySystem& system;
        const MobilizedBody& mobod;
        const string moBodyName;
    };

    class VelocityReporter : public PeriodicEventReporter {
    public:
        VelocityReporter(const MultibodySystem &system, const MobilizedBody &mobod,
                         const string moBodyName, Real reportInterval)
            : PeriodicEventReporter(reportInterval), system(system), mobod(mobod), moBodyName(moBodyName) {}
        void handleEvent(const State& state) const {
            system.realize(state, Stage::Velocity);
            Vec3 vel = mobod.getBodyOriginVelocity(state);
            cout << "Velocity of "<< moBodyName << "= { x = " << vel[0] << ", y = " << vel[1]
                 << ", z = "<< vel[2] << " }" << endl;
        }
    private:
        const MultibodySystem& system;
        const MobilizedBody& mobod;
        const string moBodyName;

    };

    class AngleReporter : public PeriodicEventReporter {
    public:
        AngleReporter(string coordinateName, Model* model, const MultibodySystem &system, Real reportInterval) :
            PeriodicEventReporter(reportInterval), model(model),
            coordinateName(coordinateName), system(system) {}
        void handleEvent(const State& state) const {
            system.realize(state, Stage::Position);
            const Coordinate& coord = model->getCoordinateSet().get(coordinateName);
            double angle = coord.getValue(state);
            cout << "angle position: " << angle << endl;

        }

    private:
        string coordinateName;
        Model* model;
        const MultibodySystem& system;
    };

}
#endif // PRINTOPENSIMINFORMATION_H

