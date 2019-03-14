#ifndef OBJECTIVE_MANAGER
#define OBJECTIVE_MANAGER

#include <vector>

enum Objectives {
    FIND_FIRE = 0,
    FIND_FOOD = 1,
    FIND_SURVIVORS = 2,
    FIND_PERSON = 3,
    RETURN_HOME = 4
};

class ObjectiveManager {
public:
    std::vector<Objectives> getActiveObjectives();
private:
}
