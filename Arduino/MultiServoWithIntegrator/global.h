#ifndef HEDGEHOG_H
#define HEDGEHOG_H


// hedgehog
float hedgehog_x = 0.; // x coordinate of hedgehog
float hedgehog_y = 0.;
float hedgehog_z = 0.;
bool  hedgehog_pos_updated = false; // flag of new data from hedgehog received
bool  high_resolution_mode = false;
unsigned int hedgehog_paired_heading = 0.; // diection of paired hedgehog
int64_t hedgehog_pos_timestamp;

uint8_t hedgehog_count = 0.;
uint8_t hedgehog_vel_count = 0.;

#endif
