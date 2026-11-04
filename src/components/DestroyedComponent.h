#pragma once

// Empty marker: an entity that has been hit and should stop being driven.
// The flight, guidance, and sensor systems all skip entities that have it,
// freezing them where they were at impact.
struct DestroyedComponent {};
