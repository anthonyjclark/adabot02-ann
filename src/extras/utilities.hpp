

#ifndef UGV_UTILITIES_HPP
#define UGV_UTILITIES_HPP


// Centimers conversion from SI units
constexpr long double operator"" _cm (long double meters) {
    return meters * 0.01;
}
constexpr long double operator"" _cm (unsigned long long meters) {
    return meters * 0.01;
}


// Notation for SI density
constexpr long double operator"" _kg_per_m3 (long double density) {
    return density;
}
constexpr long double operator"" _kg_per_m3 (unsigned long long density) {
    return density;
}


#endif
