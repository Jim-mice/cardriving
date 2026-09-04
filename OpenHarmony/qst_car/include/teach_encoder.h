#ifndef QST_CAR_TEACH_ENCODER_H
#define QST_CAR_TEACH_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#define TEACH_ENCODER_TEST_MODE 1

typedef struct {
    bool recording;
    int32_t start_left;
    int32_t start_right;
    uint32_t sample_count;
    uint32_t start_time;
} TeachEncoderState;

void TeachEncoderInit(void);
void TeachEncoderStart(void);
void TeachEncoderStop(void);
void TeachEncoderStep(uint32_t now_ms);
bool TeachEncoderIsRecording(void);

#endif
