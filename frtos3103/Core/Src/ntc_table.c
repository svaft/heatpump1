#include "ntc_table.h"



/* Таблица суммарного значения АЦП в зависимости от температуры. От большего значения к меньшему
   Для построения таблицы использованы следующие парамертры:
     R1(T1): 10кОм(25°С)
     Таблица R/T характеристик: EPCOS R/T:4001; B25/100:3950K
     Схема включения: A
     Ra: 10кОм
     Напряжения U0/Uref: 3.3В/3.3В
*/
/*
const temperature_table_entry_type termo_table[] = {
    50809, 50520, 50227, 49932, 49631, 49327, 49021, 48712,
    48400, 48086, 47769, 47450, 47128, 46804, 46476, 46147,
    45815, 45482, 45146, 44808, 44469, 44128, 43786, 43442,
    43096, 42748, 42400, 42050, 41699, 41347, 40995, 40641,
    40287, 39932, 39575, 39218, 38860, 38502, 38144, 37785,
    37427, 37069, 36711, 36353, 35992, 35631, 35270, 34910,
    34551, 34192, 33835, 33478, 33122, 32768, 32404, 32042,
    31681, 31321, 30963, 30607, 30252, 29899, 29548, 29199,
    28859, 28522, 28187, 27854, 27522, 27193, 26867, 26542,
    26220, 25900, 25582, 25266, 24952, 24641, 24333, 24027,
    23723, 23423, 23125, 22829, 22535, 22244, 21955, 21670,
    21387, 21107, 20829, 20555, 20283, 20014, 19747, 19483,
    19222, 18964, 18708, 18456, 18206, 17959, 17715, 17474,
    17233, 16995, 16760, 16528, 16299, 16072, 15849, 15628,
    15410, 15195, 14982, 14771, 14564, 14359, 14157, 13958,
    13761, 13567, 13375, 13186, 13000, 12816, 12635, 12456
};
*/
const temperature_table_entry_type termo_table[] = {
    3176, 3157, 3139, 3121, 3102, 3083, 3064, 3044,
    3025, 3005, 2986, 2966, 2945, 2925, 2905, 2884,
    2863, 2843, 2822, 2801, 2779, 2758, 2737, 2715,
    2693, 2672, 2650, 2628, 2606, 2584, 2562, 2540,
    2518, 2496, 2473, 2451, 2429, 2406, 2384, 2362,
    2339, 2317, 2294, 2272, 2249, 2227, 2204, 2182,
    2159, 2137, 2115, 2092, 2070, 2048, 2025, 2003,
    1980, 1958, 1935, 1913, 1891, 1869, 1847, 1825,
    1804, 1783, 1762, 1741, 1720, 1700, 1679, 1659,
    1639, 1619, 1599, 1579, 1560, 1540, 1521, 1502,
    1483, 1464, 1445, 1427, 1408, 1390, 1372, 1354,
    1337, 1319, 1302, 1285, 1268, 1251, 1234, 1218,
    1201, 1185, 1169, 1153, 1138, 1122, 1107, 1092,
    1077, 1062, 1048, 1033, 1019, 1005, 991, 977,
    963, 950, 936, 923, 910, 897, 885, 872,
    860, 848, 836, 824, 813, 801, 790, 779
};

// Функция вычисляет значение температуры в десятых долях градусов Цельсия
// в зависимости от суммарного значения АЦП.
int16_t calc_temperature(temperature_table_entry_type adcsum) {
  temperature_table_index_type l = 0;
  temperature_table_index_type r = (sizeof(termo_table) / sizeof(termo_table[0])) - 1;
  temperature_table_entry_type thigh = termo_table[r];  // TEMPERATURE_TABLE_READ(r);
  
  // Проверка выхода за пределы и граничных значений
  if (adcsum <= thigh) {
    #ifdef TEMPERATURE_UNDER
      if (adcsum < thigh) 
        return TEMPERATURE_UNDER;
    #endif
    return TEMPERATURE_TABLE_STEP * r + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type tlow = termo_table[0]; //TEMPERATURE_TABLE_READ(0);
  if (adcsum >= tlow) {
    #ifdef TEMPERATURE_OVER
      if (adcsum > tlow)
        return TEMPERATURE_OVER;
    #endif
    return TEMPERATURE_TABLE_START;
  }

  // Двоичный поиск по таблице
  while ((r - l) > 1) {
    temperature_table_index_type m = (l + r) >> 1;
    temperature_table_entry_type mid = termo_table[m];//TEMPERATURE_TABLE_READ(m);
    if (adcsum > mid) {
      r = m;
    } else {
      l = m;
    }
  }
  temperature_table_entry_type vl = termo_table[l];//TEMPERATURE_TABLE_READ(l);
  if (adcsum >= vl) {
    return l * TEMPERATURE_TABLE_STEP + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type vr = termo_table[r];//TEMPERATURE_TABLE_READ(r);
  temperature_table_entry_type vd = vl - vr;
  int16_t res = TEMPERATURE_TABLE_START + r * TEMPERATURE_TABLE_STEP; 
  if (vd) {
    // Линейная интерполяция
    res -= ((TEMPERATURE_TABLE_STEP * (int32_t)(adcsum - vr) + (vd >> 1)) / vd);
  }
  return res;
}