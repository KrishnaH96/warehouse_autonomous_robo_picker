#include "DetectionAlgorithm.hpp"

#include "PickObject.hpp"
#include "RobotAligner.hpp"



DetectionAlgorithm::DetectionAlgorithm(Context&  contextPtr)
: State(contextPtr) {
    
}


 bool DetectionAlgorithm::detectGoods() {

 }

   
void DetectionAlgorithm::execute() {

    if (!detectGoods()) {
        /**
         * @brief Look for Aruco codes and rotate robot if not code is not detected
         * 
         */
        std::shared_ptr<RobotAligner> rotateInplace(new RobotAligner(contextPtr_));
        contextPtr_.setState(rotateInplace);
    }else
    {
        /**
         * @brief if Aruco code is detected, pick the object
         * 
         * @return std::shared_ptr<PickObject> 
         */
        std::shared_ptr<PickObject> pickObject(new PickObject(contextPtr_));
        contextPtr_.setState(pickObject);
    }
}