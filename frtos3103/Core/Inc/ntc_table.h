#include "main.h"
//#include "pgmspace.h"

// Значение температуры, возвращаемое если сумма результатов АЦП больше первого значения таблицы
#define TEMPERATURE_UNDER -15
// Значение температуры, возвращаемое если сумма результатов АЦП меньше последнего значения таблицы
#define TEMPERATURE_OVER 620
// Значение температуры соответствующее первому значению таблицы
#define TEMPERATURE_TABLE_START -15
// Шаг таблицы 
#define TEMPERATURE_TABLE_STEP 5


// Тип каждого элемента в таблице, если сумма выходит в пределах 16 бит - uint16_t, иначе - uint32_t
typedef uint16_t temperature_table_entry_type;
// Тип индекса таблицы. Если в таблице больше 256 элементов, то uint16_t, иначе - uint8_t
typedef uint16_t temperature_table_index_type;
// Метод доступа к элементу таблицы, должна соответствовать temperature_table_entry_type


extern const temperature_table_entry_type termo_table[];
 
int16_t calc_temperature(temperature_table_entry_type adcsum);