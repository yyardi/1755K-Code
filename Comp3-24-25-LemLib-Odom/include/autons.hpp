#pragma once

enum class AutonRoutine {
    BLUE_NEGATIVE,
    RED_NEGATIVE,
    RED_POSITIVE,   
    BLUE_POSITIVE,
    SKILLS,
    AUTON_EXAMPLE,
    NONE
};

// Class to handle autonomous selection
class AutonSelector {
public:
    static AutonSelector& getInstance() {
        static AutonSelector instance;
        return instance;
    }

    void init();
    void update();
    void runSelectedAuton();
    void toggleDisplay();
    AutonRoutine getSelectedAuton() { return currentAuton; }

private:
    AutonSelector() : currentAuton(AutonRoutine::NONE), showingCoords(false) {}

    void displayAutonSelection();
    void displayCoordinates();

    AutonRoutine currentAuton;
    bool showingCoords;
    const char* getAutonName();
};
