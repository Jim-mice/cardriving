#ifndef QST_CAR_TEACH_FOLLOW_H
#define QST_CAR_TEACH_FOLLOW_H

typedef enum {
    TEACH_FOLLOW_IDLE = 0,
    TEACH_FOLLOW_RUNNING,
    TEACH_FOLLOW_DONE
} TeachFollowState;

int TeachFollowInit(void);
int TeachFollowStart(void);
TeachFollowState TeachFollowGetState(void);

#endif
