#pragma once
#include "dataStructs.h"

float get_param(char *process, char *param);
void logging(char *type, char *message);
int max(int, int);
void tokenization(struct pos *arr_to_fill, char *to_tokenize, int *objects_num);
void remove_target(int index, struct pos *objects_arr, int objects_num);