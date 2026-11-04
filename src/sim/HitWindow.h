#pragma once

// M-of-N binary integration over a rolling window of looks, stored as the
// low `size` bits of an unsigned integer: newest look at the LSB, the
// oldest aged out by the mask. Unfilled high bits read as misses, so during
// warm-up the count is what a shorter window would give. Shared by the
// airborne radar (one bit per frame) and the missile's active seeker (one
// per sweep).
namespace HitWindow {

template <typename Bits>
void Push(Bits& window, bool hit, int size) {
    Bits mask = static_cast<Bits>((1u << size) - 1u);
    window = static_cast<Bits>(((window << 1) | (hit ? 1u : 0u)) & mask);
}

template <typename Bits>
int Count(Bits window) {
    int count = 0;
    for (Bits bits = window; bits != 0; bits >>= 1) count += static_cast<int>(bits & 1u);
    return count;
}

}  // namespace HitWindow
