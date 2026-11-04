#include "sim/DragTable.h"

#include "sim/TableLookup.h"

double DragTable::Lookup(double mach) const {
    return TableLookup::Lookup(points_, mach, &Point::mach, &Point::cd);
}
