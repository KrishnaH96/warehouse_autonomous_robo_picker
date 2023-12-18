#include "Constants.hpp"


Constants* Constants::instance_;

Constants& Constants::getInstance() {
    if (Constants::instance_ == nullptr) {
        Constants::instance_ = new Constants();
    }
    return *Constants::instance_;

}