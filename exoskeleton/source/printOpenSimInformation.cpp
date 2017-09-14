#include "printOpenSimInformation.h"

namespace OpenSim {

    // PrintToFile
    void PrintToFile::PrintDataToFile(std::vector<string> nameData, std::vector<double> data)
    {
        static bool realized = false;
        if(!realized)
        {
            for(int i = 0; i < nameData.size(); i++)
                file << nameData[i] << "\t";
            file << endl;
            realized = true;
        }
        for(int i = 0; i < data.size(); i++)
            file << data[i] << "\t";
        file << endl;
    }

    void PrintToFile::PrintDataToFile(string* nameData, double* data, int size)
    {
        static bool realized = false;
        if(!realized)
        {
            for(int i = 0; i < size; i++)
                file << nameData[i] << "\t";
            file << endl;
            realized = true;
        }
        for(int i = 0; i < size; i++)
            file << data[i] << "\t";
        file << endl;

    }

    // PrintInformation Class
    void PrintInformation::printVector(const SimTK::Vector &vec)
    {
      stream << vec.toString() << endl;
    }

    void PrintInformation::printVec3(const SimTK::Vec3& vec)
    {

        stream << "[" << vec[0]
                 << ", " << vec[1]
                 << ", " << vec[2]
                 << "]" << endl;
    }

    void PrintInformation::printInertia(const SimTK::Inertia &inertia)
    {
        SimTK::Mat33 fMInertia(inertia.toMat33());
        stream << "[" << fMInertia[0][0] <<", " << fMInertia[0][1] << ", " << fMInertia[0][2] << ";" << "\n"
               << fMInertia[1][0] <<", " << fMInertia[1][1] << ", " << fMInertia[1][2] << ";" << "\n"
               << fMInertia[2][0] <<", " << fMInertia[2][1] << ", " << fMInertia[2][2] << "]" << endl;

    }

    void PrintInformation::printSpatialMat(const SimTK::SpatialMat &spatialMat, bool matlab)
    {
        /**
         *
         *
         *     | m0000, m0001, m0002, m0100, m0101, m0102 |
         *     | m0010, m0011, m0012, m0110, m0111, m0112 |
         * m = | m0020, m0021, m0022, m0120, m0121, m0122 |
         *     | m1000, m1001, m1002, m1100, m1101, m1102 |
         *     | m1010, m1011, m1012, m1110, m1111, m1112 |
         *     | m1020, m1021, m1022, m1120, m1121, m1122 |
         *
         *
         */
        const SimTK::SpatialMat &m(spatialMat);
        if (matlab) {
        stream << "["
                   << m[0][0][0][0] << "," << m[0][0][0][1] << "," << m[0][0][0][2] << ","
                        << m[0][1][0][0] << "," << m[0][1][0][1] << "," << m[0][1][0][2] << ";" << "\n" // First matrix row

                   << m[0][0][1][0] << "," << m[0][0][1][1] << "," << m[0][0][1][2] << ","
                        << m[0][1][1][0] << "," << m[0][1][1][1] << "," << m[0][1][1][2] << ";" << "\n" // Second matrix row

                   << m[0][0][2][0] << "," << m[0][0][2][1] << "," << m[0][0][2][2] << ","
                        << m[0][1][2][0] << "," << m[0][1][2][1] << "," << m[0][1][2][2] << ";" << "\n" // Third matrix row

                   << m[1][0][0][0] << "," << m[1][0][0][1] << "," << m[1][0][0][2] << ","
                       << m[1][1][0][0] << "," << m[1][1][0][1] << "," << m[1][1][0][2] << ";" << "\n" // Fourth matrix row

                   << m[1][0][1][0] << "," << m[1][0][1][1] << "," << m[1][0][1][2] << ","
                       << m[1][1][1][0] << "," << m[1][1][1][1] << "," << m[1][1][1][2] << ";" << "\n" // Fith matrix row

                   << m[1][0][2][0] << "," << m[1][0][2][1] << "," << m[1][0][2][2] << ","
                       << m[1][1][2][0] << "," << m[1][1][2][1] << "," << m[1][1][2][2] << "]" << endl;// Sith matrix row
        }
        else
        {
            stream << m.toString() << endl;
        }

    }

    // PrintModelInformation Class
    void PrintModelInformation::printCoordinateNames()
    {
        const CoordinateSet &coordinates(aModel.getCoordinateSet());
        Array<string> names;
        coordinates.getNames(names);
        stream << "COORDINATES (" << names.getSize() << ")" <<endl;
        for(int i = 0; i < names.getSize(); i++)
            stream << "coordinate[" << i << "] = "
                   << names[i] << endl;

    }

    // PrintJointInformation Class

    void PrintJointInformation::printJointNames()
    {
        for(int i = 0; i < arrayOfJointNames.getSize(); i++)
            stream << i << "- " << arrayOfJointNames[i] << endl;
    }

    void PrintJointInformation::printJointParentLocation(int index)
    {
        SimTK::Vec3 locationInParent;
        joints.get(index).getLocationInParent(locationInParent);
        stream << index << "- " << arrayOfJointNames[index] << "'s location in parent: ";
        printVec3(locationInParent);
    }

    void PrintJointInformation::printJointChildLocation(int index)
    {
        SimTK::Vec3 locationInChild;
        locationInChild = joints.get(index).getLocationInChild();
        stream << index << "- " << arrayOfJointNames[index] << "'s location in child: ";
        printVec3(locationInChild);
    }

    void PrintJointInformation::printJointParentOrientation(int index)
    {
        SimTK::Vec3 orientationInParent;
        joints.get(index).getOrientationInParent(orientationInParent);
        stream << index << "- " << arrayOfJointNames[index] << "'s orientation in parent: ";
        printVec3(orientationInParent);
    }

    void PrintJointInformation::printAllJointParentLocations()
    {
        SimTK::Vec3 locationInParent;
        for (int index = 0; index < joints.getSize(); index++)
        {
            joints.get(index).getLocationInParent(locationInParent);
            stream << index << "- " << arrayOfJointNames[index] <<"'s location in parent: ";
            printVec3(locationInParent);
        }
    }

    void PrintJointInformation::printAllJointParentOrientation()
    {
        SimTK::Vec3 orientationInParent;
        for (int index = 0; index < joints.getSize(); index++)
        {
            joints.get(index).getOrientationInParent(orientationInParent);
            stream << index << "- " << arrayOfJointNames[index] <<"'s location in parent: ";
            printVec3(orientationInParent);
        }

    }

    // PrintForceToFile Class
    void PrintForceToFile::getForces() {
        forceReporter->getForceStorage().getDataForIdentifier(forceName, rData);
    }

    void PrintForceToFile::getForceNames() {
        forceNames = forceReporter->getForceStorage().getColumnLabels();
    }

    void PrintForceToFile::printForces() {
        getForces();
        getForceNames();
        int firstIndex, lastIndex;
        bool firstTime = false, forceHasBeenFound = false;
        file << "This file print the force that are interested on:" << '\n';
        for(int i = 0; i < forceNames.getSize(); i++) {
            size_t found = forceNames[i].find(forceName);
            if (found != string::npos) {
                if(!firstTime) {
                    firstIndex = i;
                    forceHasBeenFound = true;
                    firstTime = false;
                }
                file << forceNames[i] << '\t';
            }
            else if(forceHasBeenFound) {
                file << '\n';
                forceHasBeenFound = false;
                lastIndex = i;
            }
        }
        for(int j = 0; j < rData[0].getSize(); j++) {
            for(int i = 0; i < rData.getSize(); i++) {
                file << std::fixed << std::setprecision(3) << rData[i][j] << '\t';
            }
            file << '\n';
        }

    file.close();
    }

}

//int main()
//{

//    return 0;
//}


