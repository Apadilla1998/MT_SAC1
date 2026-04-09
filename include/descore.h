#ifndef DESCORE_H
#define DESCORE_H

// Initialize the descore mechanism
void initDescore();

// Update descore target
// deployed = true  -> move to descore position
// deployed = false -> return to home position
void updateDescore(bool deployed);

// Print descore angle to controller screen
void printDescoreAngleControllerUpdate();

#endif