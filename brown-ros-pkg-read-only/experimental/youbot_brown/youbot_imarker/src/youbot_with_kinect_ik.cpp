
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#ifdef IKFAST_HEADER
#include IKFAST_HEADER
#endif

#define IKFAST_NO_MAIN

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI  ((IKReal)6.28318530717959)
#define IKPI  ((IKReal)3.14159265358979)
#define IKPI_2  ((IKReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#endif // _MSC_VER

// defined when creating a shared object/dll
#ifdef IKFAST_CLIBRARY
#ifdef _MSC_VER
#define IKFAST_API extern "C" __declspec(dllexport)
#else
#define IKFAST_API extern "C"
#endif
#else
#define IKFAST_API
#endif

// lapack routines
extern "C" {
  void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
  void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
  void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
  void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
  void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
  void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

#ifdef IKFAST_REAL
typedef IKFAST_REAL IKReal;
#else
typedef double IKReal;
#endif

class IKSolution
{
public:
    /// Gets a solution given its free parameters
    /// \param pfree The free parameters required, range is in [-pi,pi]
    void GetSolution(IKReal* psolution, const IKReal* pfree) const {
        for(std::size_t i = 0; i < basesol.size(); ++i) {
            if( basesol[i].freeind < 0 )
                psolution[i] = basesol[i].foffset;
            else {
                IKFAST_ASSERT(pfree != NULL);
                psolution[i] = pfree[basesol[i].freeind]*basesol[i].fmul + basesol[i].foffset;
                if( psolution[i] > IKPI ) {
                    psolution[i] -= IK2PI;
                }
                else if( psolution[i] < -IKPI ) {
                    psolution[i] += IK2PI;
                }
            }
        }
    }

    /// Gets the free parameters the solution requires to be set before a full solution can be returned
    /// \return vector of indices indicating the free parameters
    const std::vector<int>& GetFree() const { return vfree; }

    struct VARIABLE
    {
        VARIABLE() : fmul(0), foffset(0), freeind(-1), maxsolutions(1) {
            indices[0] = indices[1] = -1;
        }
        IKReal fmul, foffset; ///< joint value is fmul*sol[freeind]+foffset
        signed char freeind; ///< if >= 0, mimics another joint
        unsigned char maxsolutions; ///< max possible indices, 0 if controlled by free index or a free joint itself
        unsigned char indices[2]; ///< unique index of the solution used to keep track on what part it came from. sometimes a solution can be repeated for different indices. store at least another repeated root
    };

    std::vector<VARIABLE> basesol;       ///< solution and their offsets if joints are mimiced
    std::vector<int> vfree;

    bool Validate() const {
        for(size_t i = 0; i < basesol.size(); ++i) {
            if( basesol[i].maxsolutions == (unsigned char)-1) {
                return false;
            }
            if( basesol[i].maxsolutions > 0 ) {
                if( basesol[i].indices[0] >= basesol[i].maxsolutions ) {
                    return false;
                }
                if( basesol[i].indices[1] != (unsigned char)-1 && basesol[i].indices[1] >= basesol[i].maxsolutions ) {
                    return false;
                }
            }
        }
        return true;
    }

    void GetSolutionIndices(std::vector<unsigned int>& v) const {
        v.resize(0);
        v.push_back(0);
        for(int i = (int)basesol.size()-1; i >= 0; --i) {
            if( basesol[i].maxsolutions != (unsigned char)-1 && basesol[i].maxsolutions > 1 ) {
                for(size_t j = 0; j < v.size(); ++j) {
                    v[j] *= basesol[i].maxsolutions;
                }
                size_t orgsize=v.size();
                if( basesol[i].indices[1] != (unsigned char)-1 ) {
                    for(size_t j = 0; j < orgsize; ++j) {
                        v.push_back(v[j]+basesol[i].indices[1]);
                    }
                }
                if( basesol[i].indices[0] != (unsigned char)-1 ) {
                    for(size_t j = 0; j < orgsize; ++j) {
                        v[j] += basesol[i].indices[0];
                    }
                }
            }
        }
    }
};

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IKReal)0.000001)
#endif

// used to check input to atan2 for degenerate cases
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IKReal)2e-6)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IKReal)1e-6)
#endif

inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

/// solves the forward kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API void fk(const IKReal* j, IKReal* eetrans, IKReal* eerot) {
IKReal x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18;
x0=IKcos(j[1]);
x1=IKsin(j[1]);
x2=((0.422620344255005)*(x0));
x3=((0.906306815941368)*(x1));
x4=((x3)+(x2));
x5=IKsin(j[2]);
x6=IKcos(j[2]);
x7=((0.422620344255005)*(x1));
x8=((0.906306815941368)*(x0));
x9=((x7)+(((-1.00000000000000)*(x8))));
x10=((x8)+(((-1.00000000000000)*(x7))));
x11=IKsin(j[3]);
x12=IKcos(j[3]);
x13=((0.559191561564770)*(x4)*(x5));
x14=((0.829038477680472)*(x4)*(x6));
x15=((0.829038477680472)*(x10)*(x5));
x16=((0.559191561564770)*(x6)*(x9));
x17=((x13)+(x15)+(x14)+(x16));
x18=((-1.00000000000000)*(x17));
eetrans[0]=((0.0240000000000000)+(((((((-0.0243938659818467)*(IKsin(j[0]))))+(((0.138343352476720)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((0.00573043550602780)*(IKsin(j[0]))))+(((-0.0324986478012741)*(IKcos(j[0]))))+(((((((0.0754908608112439)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((0.0754908608112439)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))+(((((((0.0300896801891500)*(((((-0.829038477680472)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.829038477680472)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((0.0300896801891500)*(((((-0.559191561564770)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.559191561564770)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))+(((0.0300896801891500)*(((((-0.829038477680472)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.829038477680472)*(((((0.892537757914323)*(IKcos(j[0]))))+(((-0.157379780528043)*(IKsin(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))+(((0.0300896801891500)*(((((-0.559191561564770)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.559191561564770)*(((((-0.416199688533230)*(IKcos(j[0]))))+(((0.0733878371602598)*(IKsin(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))))*(IKcos(j[3]))))+(((((((0.0754908608112439)*(((((-0.416199688533230)*(IKcos(j[0]))))+(((0.0733878371602598)*(IKsin(j[0]))))))*(IKcos(j[1]))))+(((0.0754908608112439)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKcos(j[2]))))+(((((((0.111920194486864)*(((((0.892537757914323)*(IKcos(j[0]))))+(((-0.157379780528043)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((0.111920194486864)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))+(((((((0.111920194486864)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((0.111920194486864)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKcos(j[2]))))+(((((((-0.126485616360575)*(((((-0.829038477680472)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.829038477680472)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((-0.126485616360575)*(((((-0.829038477680472)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.829038477680472)*(((((0.892537757914323)*(IKcos(j[0]))))+(((-0.157379780528043)*(IKsin(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))+(((-0.126485616360575)*(((((-0.559191561564770)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.559191561564770)*(((((-0.416199688533230)*(IKcos(j[0]))))+(((0.0733878371602598)*(IKsin(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((-0.126485616360575)*(((((-0.559191561564770)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.559191561564770)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))))*(IKsin(j[3]))))+(((((((0.0113751147598403)*(IKsin(j[0]))))+(((-0.0645109517226507)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((((((0.0300896801891500)*(((((0.829038477680472)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((0.829038477680472)*(((((-0.416199688533230)*(IKcos(j[0]))))+(((0.0733878371602598)*(IKsin(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((0.0300896801891500)*(((((-0.559191561564770)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.559191561564770)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKcos(j[2]))))+(((0.0300896801891500)*(((((0.829038477680472)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((0.829038477680472)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))+(((0.0300896801891500)*(((((-0.559191561564770)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.559191561564770)*(((((0.892537757914323)*(IKcos(j[0]))))+(((-0.157379780528043)*(IKsin(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))))*(IKsin(j[3]))))+(((((((-0.126485616360575)*(((((0.559191561564770)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((0.559191561564770)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKcos(j[2]))))+(((-0.126485616360575)*(((((-0.829038477680472)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.829038477680472)*(((((-0.416199688533230)*(IKcos(j[0]))))+(((0.0733878371602598)*(IKsin(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((-0.126485616360575)*(((((-0.829038477680472)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.829038477680472)*(((((0.157379780528043)*(IKsin(j[0]))))+(((-0.892537757914323)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))+(((-0.126485616360575)*(((((0.559191561564770)*(((((0.892537757914323)*(IKcos(j[0]))))+(((-0.157379780528043)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((0.559191561564770)*(((((-0.0733878371602598)*(IKsin(j[0]))))+(((0.416199688533230)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))))*(IKcos(j[3])))));
eetrans[1]=((((((((-0.111920194486864)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKcos(j[1]))))+(((-0.111920194486864)*(((((0.892537757914323)*(IKsin(j[0]))))+(((0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))+(((((((0.0645109517226507)*(IKsin(j[0]))))+(((0.0113751147598403)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((((((0.126485616360575)*(((((-0.829038477680472)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.829038477680472)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))+(((0.126485616360575)*(((((-0.829038477680472)*(((((-0.416199688533230)*(IKsin(j[0]))))+(((-0.0733878371602598)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.829038477680472)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKcos(j[2]))))+(((0.126485616360575)*(((((0.559191561564770)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((0.559191561564770)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((0.126485616360575)*(((((0.559191561564770)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKcos(j[1]))))+(((0.559191561564770)*(((((0.892537757914323)*(IKsin(j[0]))))+(((0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))))*(IKcos(j[3]))))+(((((((-0.138343352476720)*(IKsin(j[0]))))+(((-0.0243938659818467)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((((((0.126485616360575)*(((((-0.559191561564770)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.559191561564770)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))+(((0.126485616360575)*(((((-0.559191561564770)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.559191561564770)*(((((-0.416199688533230)*(IKsin(j[0]))))+(((-0.0733878371602598)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((0.126485616360575)*(((((-0.829038477680472)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.829038477680472)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((0.126485616360575)*(((((-0.829038477680472)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKcos(j[1]))))+(((-0.829038477680472)*(((((0.892537757914323)*(IKsin(j[0]))))+(((0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))))*(IKsin(j[3]))))+(((((((-0.111920194486864)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.111920194486864)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((((((-0.0754908608112439)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.0754908608112439)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))+(((((((-0.0300896801891500)*(((((-0.559191561564770)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((-0.559191561564770)*(((((-0.416199688533230)*(IKsin(j[0]))))+(((-0.0733878371602598)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((-0.0300896801891500)*(((((-0.559191561564770)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.559191561564770)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKsin(j[2]))))+(((-0.0300896801891500)*(((((-0.829038477680472)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.829038477680472)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((-0.0300896801891500)*(((((-0.829038477680472)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKcos(j[1]))))+(((-0.829038477680472)*(((((0.892537757914323)*(IKsin(j[0]))))+(((0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))))*(IKcos(j[3]))))+(((((((-0.0754908608112439)*(((((-0.416199688533230)*(IKsin(j[0]))))+(((-0.0733878371602598)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((-0.0754908608112439)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKcos(j[2]))))+(((((((-0.0300896801891500)*(((((-0.559191561564770)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))+(((-0.559191561564770)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((-0.0300896801891500)*(((((0.829038477680472)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))+(((0.829038477680472)*(((((-0.416199688533230)*(IKsin(j[0]))))+(((-0.0733878371602598)*(IKcos(j[0]))))))*(IKcos(j[1]))))))*(IKcos(j[2]))))+(((-0.0300896801891500)*(((((-0.559191561564770)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKcos(j[1]))))+(((-0.559191561564770)*(((((0.892537757914323)*(IKsin(j[0]))))+(((0.157379780528043)*(IKcos(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))+(((-0.0300896801891500)*(((((0.829038477680472)*(((((-0.892537757914323)*(IKsin(j[0]))))+(((-0.157379780528043)*(IKcos(j[0]))))))*(IKcos(j[1]))))+(((0.829038477680472)*(((((0.0733878371602598)*(IKcos(j[0]))))+(((0.416199688533230)*(IKsin(j[0]))))))*(IKsin(j[1]))))))*(IKsin(j[2]))))))*(IKsin(j[3]))))+(((0.0324986478012741)*(IKsin(j[0]))))+(((0.00573043550602780)*(IKcos(j[0])))));
IKReal x19=((0.829038477680472)*(x6)*(x9));
IKReal x20=((0.829038477680472)*(x4)*(x5));
IKReal x21=((x19)+(x20));
IKReal x22=((0.559191561564770)*(x10)*(x5));
IKReal x23=((0.559191561564770)*(x4)*(x6));
IKReal x24=((x22)+(x23));
eetrans[2]=((0.115000000000000)+(((-0.0754908608112439)*(x4)*(x5)))+(((-0.111920194486864)*(x10)*(x5)))+(((-0.111920194486864)*(x4)*(x6)))+(((0.0655061533595257)*(x0)))+(((-0.0300896801891500)*(x12)*(x18)))+(((-0.0754908608112439)*(x6)*(x9)))+(((0.126485616360575)*(x11)*(x18)))+(((0.126485616360575)*(x12)*(((((-1.00000000000000)*(x21)))+(x24)))))+(((0.140477556470912)*(x1)))+(((-0.0300896801891500)*(x11)*(((((-1.00000000000000)*(x24)))+(x21))))));
}

IKFAST_API int getNumFreeParameters() { return 2; }
IKFAST_API int* getFreeParameters() { static int freeparams[] = {3, 4}; return freeparams; }
IKFAST_API int getNumJoints() { return 5; }

IKFAST_API int getIKRealSize() { return sizeof(IKReal); }

IKFAST_API int getIKType() { return 0x33000003; }

class IKSolver {
public:
IKReal j0,cj0,sj0,htj0,j1,cj1,sj1,htj1,j2,cj2,sj2,htj2,j3,cj3,sj3,htj3,j4,cj4,sj4,htj4,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij0[2], _nj0,_ij1[2], _nj1,_ij2[2], _nj2,_ij3[2], _nj3,_ij4[2], _nj4;

bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions) {
j0=numeric_limits<IKReal>::quiet_NaN(); _ij0[0] = -1; _ij0[1] = -1; _nj0 = -1; j1=numeric_limits<IKReal>::quiet_NaN(); _ij1[0] = -1; _ij1[1] = -1; _nj1 = -1; j2=numeric_limits<IKReal>::quiet_NaN(); _ij2[0] = -1; _ij2[1] = -1; _nj2 = -1;  _ij3[0] = -1; _ij3[1] = -1; _nj3 = 0;  _ij4[0] = -1; _ij4[1] = -1; _nj4 = 0; 
for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {
    vsolutions.resize(0); vsolutions.reserve(8);
j3=pfree[0]; cj3=cos(pfree[0]); sj3=sin(pfree[0]);
j4=pfree[1]; cj4=cos(pfree[1]); sj4=sin(pfree[1]);
px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];

new_px=((-0.0240000000000000)+(px));
new_py=((-1.00000000000000)*(py));
new_pz=((0.115000000000000)+(((-1.00000000000000)*(pz))));
px = new_px; py = new_py; pz = new_pz;
pp=(((px)*(px))+((py)*(py))+((pz)*(pz)));
{
IKReal j0array[2], cj0array[2], sj0array[2];
bool j0valid[2]={false};
_nj0 = 2;
IKReal x25=((0.173649560788721)*(px));
IKReal x26=((0.984807509129517)*(py));
IKReal x27=((((-1.00000000000000)*(x26)))+(x25));
IKReal x28=((0.984807509129517)*(px));
IKReal x29=((0.173649560788721)*(py));
IKReal x30=((x28)+(x29));
if( IKabs(x27) < IKFAST_ATAN2_MAGTHRESH && IKabs(x30) < IKFAST_ATAN2_MAGTHRESH )
    continue;
IKReal x31=IKatan2(x27, x30);
j0array[0]=((-1.00000000000000)*(x31));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
j0array[1]=((3.14159265358979)+(((-1.00000000000000)*(x31))));
sj0array[1]=IKsin(j0array[1]);
cj0array[1]=IKcos(j0array[1]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
if( j0array[1] > IKPI )
{
    j0array[1]-=IK2PI;
}
else if( j0array[1] < -IKPI )
{    j0array[1]+=IK2PI;
}
j0valid[1] = true;
for(int ij0 = 0; ij0 < 2; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 2; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];

{
IKReal dummyeval[1];
dummyeval[0]=((2.24329403222902)+(((2.08069368015360)*((sj3)*(sj3))))+(((-1.00000000000000)*(cj3)))+(((4.20362116065909)*(sj3)))+(((2.08069368015360)*((cj3)*(cj3)))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
{
IKReal dummyeval[1];
dummyeval[0]=((2.24329403222902)+(((2.08069368015360)*((sj3)*(sj3))))+(((-1.00000000000000)*(cj3)))+(((4.20362116065909)*(sj3)))+(((2.08069368015360)*((cj3)*(cj3)))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
{
IKReal dummyeval[1];
IKReal x32=(py)*(py);
IKReal x33=(cj0)*(cj0);
IKReal x34=(px)*(px);
IKReal x35=(sj0)*(sj0);
dummyeval[0]=((1.00000000000000)+(((1725.78817277460)*(cj0)*(px)*(py)*(sj0)))+(((-314.070507661663)*(cj0)*(sj0)*(x34)))+(((314.070507661663)*(cj0)*(sj0)*(x32)))+(((27.6897795795370)*(x34)*(x35)))+(((314.070507661663)*(px)*(py)*(x33)))+(((59.6853035836071)*(py)*(sj0)))+(((918.273645546373)*((pz)*(pz))))+(((890.583865966836)*(x33)*(x34)))+(((27.6897795795370)*(x32)*(x33)))+(((59.6853035836071)*(cj0)*(px)))+(((890.583865966836)*(x32)*(x35)))+(((-10.5242158053771)*(px)*(sj0)))+(((-314.070507661663)*(px)*(py)*(x35)))+(((10.5242158053771)*(cj0)*(py))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
continue;

} else
{
{
IKReal j1array[2], cj1array[2], sj1array[2];
bool j1valid[2]={false};
_nj1 = 2;
IKReal x36=((0.280955112941824)*(pz));
IKReal x37=((0.129021903445301)*(cj0)*(px));
IKReal x38=((0.129021903445301)*(py)*(sj0));
IKReal x39=((0.0227502295196805)*(cj0)*(py));
IKReal x40=((0.0227502295196805)*(px)*(sj0));
IKReal x41=((0.0487877319636934)*(cj0)*(py));
IKReal x42=((0.276686704953440)*(cj0)*(px));
IKReal x43=((0.276686704953440)*(py)*(sj0));
IKReal x44=((0.00927151872708020)+(x42)+(x43)+(x41));
IKReal x45=((0.131012306719051)*(pz));
IKReal x46=((0.0487877319636934)*(px)*(sj0));
IKReal x47=((x46)+(x45));
IKReal x48=((((-1.00000000000000)*(x47)))+(x44));
IKReal x49=((0.00432340612172870)+(x39)+(x38)+(x37)+(x36));
IKReal x50=((((-1.00000000000000)*(x40)))+(x49));
IKReal x51=(x50)*(x50);
IKReal x52=(x48)*(x48);
IKReal x53=((x51)+(x52));
if( (x53) < (IKReal)-0.00001 )
    continue;
IKReal x54=IKsqrt(x53);
IKReal x55=IKabs(x54);
IKReal x56=((IKabs(x55) != 0)?((IKReal)1/(x55)):(IKReal)1.0e30);
IKReal x57=((0.0114608710120556)*(px)*(sj0));
IKReal x58=((0.0341511164173554)*(sj3));
IKReal x59=((0.0100150000000000)+(x58)+(x57));
IKReal x60=((0.0649972956025481)*(py)*(sj0));
IKReal x61=((0.00812421365107049)*(cj3));
IKReal x62=((0.0114608710120556)*(cj0)*(py));
IKReal x63=((0.0649972956025481)*(cj0)*(px));
IKReal x64=((pp)+(x60)+(x61)+(x62)+(x63));
IKReal x65=((x59)+(((-1.00000000000000)*(x64))));
IKReal x66=((x56)*(x65));
if( (x66) < -1-IKFAST_SINCOS_THRESH || (x66) > 1+IKFAST_SINCOS_THRESH )
    continue;
IKReal x67=IKasin(x66);
IKReal x68=((-0.00432340612172870)+(x40));
IKReal x69=((-0.00432340612172870)+(x49));
IKReal x70=((x68)+(((-1.00000000000000)*(x69))));
if( IKabs(x48) < IKFAST_ATAN2_MAGTHRESH && IKabs(x70) < IKFAST_ATAN2_MAGTHRESH )
    continue;
IKReal x71=IKatan2(x48, x70);
j1array[0]=((((-1.00000000000000)*(x71)))+(((-1.00000000000000)*(x67))));
sj1array[0]=IKsin(j1array[0]);
cj1array[0]=IKcos(j1array[0]);
j1array[1]=((3.14159265358979)+(((-1.00000000000000)*(x71)))+(x67));
sj1array[1]=IKsin(j1array[1]);
cj1array[1]=IKcos(j1array[1]);
if( j1array[0] > IKPI )
{
    j1array[0]-=IK2PI;
}
else if( j1array[0] < -IKPI )
{    j1array[0]+=IK2PI;
}
j1valid[0] = true;
if( j1array[1] > IKPI )
{
    j1array[1]-=IK2PI;
}
else if( j1array[1] < -IKPI )
{    j1array[1]+=IK2PI;
}
j1valid[1] = true;
for(int ij1 = 0; ij1 < 2; ++ij1)
{
if( !j1valid[ij1] )
{
    continue;
}
_ij1[0] = ij1; _ij1[1] = -1;
for(int iij1 = ij1+1; iij1 < 2; ++iij1)
{
if( j1valid[iij1] && IKabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && IKabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
{
    j1valid[iij1]=false; _ij1[1] = iij1; break; 
}
}
j1 = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];

{
IKReal dummyeval[1];
IKReal gconst0;
IKReal x72=(cj3)*(cj3);
IKReal x73=(sj3)*(sj3);
gconst0=IKsign(((((-0.00228253936390002)*(cj1)*(cj3)))+(((0.00959493077012747)*(cj1)*(sj3)))+(((-0.00238769928995471)*(sj1)))+(((0.00106437197070515)*(cj3)*(sj1)))+(((0.00474926522916860)*(cj1)*(x72)))+(((0.00474926522916860)*(cj1)*(x73)))+(((-0.00447421653886860)*(sj1)*(sj3)))+(((-0.00221463203277885)*(sj1)*(x73)))+(((-0.00221463203277885)*(sj1)*(x72)))+(((0.00512040693336474)*(cj1)))));
IKReal x74=(cj3)*(cj3);
IKReal x75=(sj3)*(sj3);
dummyeval[0]=((((-2.14449405538914)*(cj1)*(cj3)))+(((4.81073071660507)*(cj1)))+(((9.01464059014141)*(cj1)*(sj3)))+(((-4.20362116065909)*(sj1)*(sj3)))+(((-2.08069368015360)*(sj1)*(x75)))+(((-2.08069368015360)*(sj1)*(x74)))+(((4.46203522817515)*(cj1)*(x75)))+(((4.46203522817515)*(cj1)*(x74)))+(((-2.24329403222902)*(sj1)))+(((cj3)*(sj1))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
continue;

} else
{
{
IKReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((gconst0)*(((((0.00169066690488961)*(cj1)*((sj3)*(sj3))))+(((0.00486622587999134)*(cj1)*(cj3)))+(((-0.0296593095156779)*(cj3)*(pz)))+(((-0.00321772135468784)*(cj0)*(cj3)*(px)*(sj1)))+(((-0.000567375750897680)*(cj0)*(cj1)*(py)*(sj3)))+(((-0.000567375750897680)*(cj0)*(cj3)*(py)*(sj1)))+(((-0.00321772135468784)*(cj3)*(py)*(sj0)*(sj1)))+(((-0.0495054651867961)*(cj1)*(pp)*(sj3)))+(((-0.00770052745170581)*(sj1)*(sj3)))+(((0.00137784309743520)*(cj0)*(py)*(sj1)*(sj3)))+(((0.00538347229688298)*(cj1)*(sj3)))+(((-0.00137262137532159)*(cj0)*(cj1)*(px)))+(((0.133338008689083)*(pp)*(sj1)))+(((-0.120221499394405)*(cj1)*(cj3)*(pp)))+(((0.0346952602909277)*(pz)))+(((-0.00410569842168734)*(sj1)*((sj3)*(sj3))))+(((-0.00321772135468784)*(cj1)*(py)*(sj0)*(sj3)))+(((0.000567375750897680)*(cj1)*(px)*(sj0)*(sj3)))+(((-0.0211181305713861)*(cj1)*(pp)))+(((0.00781407233391970)*(py)*(sj0)*(sj1)*(sj3)))+(((0.000242032170494404)*(cj1)*(px)*(sj0)))+(((-0.00137784309743520)*(cj0)*(cj1)*(cj3)*(py)))+(((-0.0495054651867961)*(cj3)*(pp)*(sj1)))+(((-0.00137784309743520)*(px)*(sj0)*(sj1)*(sj3)))+(((-0.000976705146532187)*(cj1)*((cj3)*(cj3))))+(((0.00266737205142180)*(cj3)*(sj1)*(sj3)))+(((-0.00286836608773979)*(sj1)))+(((0.00370350544561418)*(cj1)*(cj3)*(sj3)))+(((0.0272910197510686)*(pz)*(sj3)))+(((0.00152816971858993)*(cj0)*(py)*(sj1)))+(((0.00781407233391970)*(cj0)*(px)*(sj1)*(sj3)))+(((-0.00321772135468784)*(cj0)*(cj1)*(px)*(sj3)))+(((0.120221499394405)*(pp)*(sj1)*(sj3)))+(((-0.00152816971858993)*(px)*(sj0)*(sj1)))+(((-0.000208666020907185)*(cj3)*(sj1)))+(((0.00866660996581944)*(py)*(sj0)*(sj1)))+(((0.000567375750897680)*(cj3)*(px)*(sj0)*(sj1)))+(((0.00349897729309371)*(cj1)))+(((-0.00137262137532159)*(cj1)*(py)*(sj0)))+(((0.00866660996581944)*(cj0)*(px)*(sj1)))+(((-0.000402192976073164)*(sj1)*((cj3)*(cj3))))+(((-0.000242032170494404)*(cj0)*(cj1)*(py)))+(((0.00137784309743520)*(cj1)*(cj3)*(px)*(sj0)))+(((-0.00781407233391970)*(cj1)*(cj3)*(py)*(sj0)))+(((-0.00781407233391970)*(cj0)*(cj1)*(cj3)*(px))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((gconst0)*(((((-0.0234021668514856)*(pz)))+(((-0.120221499394405)*(cj3)*(pp)*(sj1)))+(((0.00410569842168734)*(cj1)*((sj3)*(sj3))))+(((0.00152816971858993)*(cj1)*(px)*(sj0)))+(((0.000242032170494404)*(px)*(sj0)*(sj1)))+(((-0.00206125496379374)*(sj1)))+(((-0.00266737205142180)*(cj1)*(cj3)*(sj3)))+(((0.00321772135468784)*(cj1)*(cj3)*(py)*(sj0)))+(((-0.000567375750897680)*(cj1)*(cj3)*(px)*(sj0)))+(((-0.00781407233391970)*(cj3)*(py)*(sj0)*(sj1)))+(((-0.00321772135468784)*(py)*(sj0)*(sj1)*(sj3)))+(((0.00297531738943346)*(cj3)*(sj1)))+(((-0.00781407233391970)*(cj0)*(cj3)*(px)*(sj1)))+(((0.000567375750897680)*(cj0)*(cj1)*(cj3)*(py)))+(((-0.0296593095156779)*(pz)*(sj3)))+(((-0.0272910197510686)*(cj3)*(pz)))+(((-0.00866660996581944)*(cj0)*(cj1)*(px)))+(((-0.000976705146532187)*(sj1)*((cj3)*(cj3))))+(((-0.00137784309743520)*(cj0)*(cj1)*(py)*(sj3)))+(((0.00321772135468784)*(cj0)*(cj1)*(cj3)*(px)))+(((-0.0211181305713861)*(pp)*(sj1)))+(((0.00137784309743520)*(cj1)*(px)*(sj0)*(sj3)))+(((-0.000570724755645268)*(sj1)*(sj3)))+(((-0.120221499394405)*(cj1)*(pp)*(sj3)))+(((-0.00137262137532159)*(cj0)*(px)*(sj1)))+(((-0.133338008689083)*(cj1)*(pp)))+(((0.00370350544561418)*(cj3)*(sj1)*(sj3)))+(((-0.000567375750897680)*(cj0)*(py)*(sj1)*(sj3)))+(((0.0495054651867961)*(cj1)*(cj3)*(pp)))+(((-0.00137784309743520)*(cj0)*(cj3)*(py)*(sj1)))+(((0.00620928554381296)*(cj1)))+(((0.000402192976073164)*(cj1)*((cj3)*(cj3))))+(((0.00137784309743520)*(cj3)*(px)*(sj0)*(sj1)))+(((-0.00137262137532159)*(py)*(sj0)*(sj1)))+(((-0.0495054651867961)*(pp)*(sj1)*(sj3)))+(((-0.00781407233391970)*(cj0)*(cj1)*(px)*(sj3)))+(((0.000567375750897680)*(px)*(sj0)*(sj1)*(sj3)))+(((-0.00866660996581944)*(cj1)*(py)*(sj0)))+(((-0.00781407233391970)*(cj1)*(py)*(sj0)*(sj3)))+(((-0.00321772135468784)*(cj0)*(px)*(sj1)*(sj3)))+(((0.00169066690488961)*(sj1)*((sj3)*(sj3))))+(((-0.00574553103162107)*(cj1)*(cj3)))+(((-0.00152816971858993)*(cj0)*(cj1)*(py)))+(((0.00959143594226369)*(cj1)*(sj3)))+(((-0.000242032170494404)*(cj0)*(py)*(sj1))))))) < IKFAST_ATAN2_MAGTHRESH )
    continue;
j2array[0]=IKatan2(((gconst0)*(((((0.00169066690488961)*(cj1)*((sj3)*(sj3))))+(((0.00486622587999134)*(cj1)*(cj3)))+(((-0.0296593095156779)*(cj3)*(pz)))+(((-0.00321772135468784)*(cj0)*(cj3)*(px)*(sj1)))+(((-0.000567375750897680)*(cj0)*(cj1)*(py)*(sj3)))+(((-0.000567375750897680)*(cj0)*(cj3)*(py)*(sj1)))+(((-0.00321772135468784)*(cj3)*(py)*(sj0)*(sj1)))+(((-0.0495054651867961)*(cj1)*(pp)*(sj3)))+(((-0.00770052745170581)*(sj1)*(sj3)))+(((0.00137784309743520)*(cj0)*(py)*(sj1)*(sj3)))+(((0.00538347229688298)*(cj1)*(sj3)))+(((-0.00137262137532159)*(cj0)*(cj1)*(px)))+(((0.133338008689083)*(pp)*(sj1)))+(((-0.120221499394405)*(cj1)*(cj3)*(pp)))+(((0.0346952602909277)*(pz)))+(((-0.00410569842168734)*(sj1)*((sj3)*(sj3))))+(((-0.00321772135468784)*(cj1)*(py)*(sj0)*(sj3)))+(((0.000567375750897680)*(cj1)*(px)*(sj0)*(sj3)))+(((-0.0211181305713861)*(cj1)*(pp)))+(((0.00781407233391970)*(py)*(sj0)*(sj1)*(sj3)))+(((0.000242032170494404)*(cj1)*(px)*(sj0)))+(((-0.00137784309743520)*(cj0)*(cj1)*(cj3)*(py)))+(((-0.0495054651867961)*(cj3)*(pp)*(sj1)))+(((-0.00137784309743520)*(px)*(sj0)*(sj1)*(sj3)))+(((-0.000976705146532187)*(cj1)*((cj3)*(cj3))))+(((0.00266737205142180)*(cj3)*(sj1)*(sj3)))+(((-0.00286836608773979)*(sj1)))+(((0.00370350544561418)*(cj1)*(cj3)*(sj3)))+(((0.0272910197510686)*(pz)*(sj3)))+(((0.00152816971858993)*(cj0)*(py)*(sj1)))+(((0.00781407233391970)*(cj0)*(px)*(sj1)*(sj3)))+(((-0.00321772135468784)*(cj0)*(cj1)*(px)*(sj3)))+(((0.120221499394405)*(pp)*(sj1)*(sj3)))+(((-0.00152816971858993)*(px)*(sj0)*(sj1)))+(((-0.000208666020907185)*(cj3)*(sj1)))+(((0.00866660996581944)*(py)*(sj0)*(sj1)))+(((0.000567375750897680)*(cj3)*(px)*(sj0)*(sj1)))+(((0.00349897729309371)*(cj1)))+(((-0.00137262137532159)*(cj1)*(py)*(sj0)))+(((0.00866660996581944)*(cj0)*(px)*(sj1)))+(((-0.000402192976073164)*(sj1)*((cj3)*(cj3))))+(((-0.000242032170494404)*(cj0)*(cj1)*(py)))+(((0.00137784309743520)*(cj1)*(cj3)*(px)*(sj0)))+(((-0.00781407233391970)*(cj1)*(cj3)*(py)*(sj0)))+(((-0.00781407233391970)*(cj0)*(cj1)*(cj3)*(px)))))), ((gconst0)*(((((-0.0234021668514856)*(pz)))+(((-0.120221499394405)*(cj3)*(pp)*(sj1)))+(((0.00410569842168734)*(cj1)*((sj3)*(sj3))))+(((0.00152816971858993)*(cj1)*(px)*(sj0)))+(((0.000242032170494404)*(px)*(sj0)*(sj1)))+(((-0.00206125496379374)*(sj1)))+(((-0.00266737205142180)*(cj1)*(cj3)*(sj3)))+(((0.00321772135468784)*(cj1)*(cj3)*(py)*(sj0)))+(((-0.000567375750897680)*(cj1)*(cj3)*(px)*(sj0)))+(((-0.00781407233391970)*(cj3)*(py)*(sj0)*(sj1)))+(((-0.00321772135468784)*(py)*(sj0)*(sj1)*(sj3)))+(((0.00297531738943346)*(cj3)*(sj1)))+(((-0.00781407233391970)*(cj0)*(cj3)*(px)*(sj1)))+(((0.000567375750897680)*(cj0)*(cj1)*(cj3)*(py)))+(((-0.0296593095156779)*(pz)*(sj3)))+(((-0.0272910197510686)*(cj3)*(pz)))+(((-0.00866660996581944)*(cj0)*(cj1)*(px)))+(((-0.000976705146532187)*(sj1)*((cj3)*(cj3))))+(((-0.00137784309743520)*(cj0)*(cj1)*(py)*(sj3)))+(((0.00321772135468784)*(cj0)*(cj1)*(cj3)*(px)))+(((-0.0211181305713861)*(pp)*(sj1)))+(((0.00137784309743520)*(cj1)*(px)*(sj0)*(sj3)))+(((-0.000570724755645268)*(sj1)*(sj3)))+(((-0.120221499394405)*(cj1)*(pp)*(sj3)))+(((-0.00137262137532159)*(cj0)*(px)*(sj1)))+(((-0.133338008689083)*(cj1)*(pp)))+(((0.00370350544561418)*(cj3)*(sj1)*(sj3)))+(((-0.000567375750897680)*(cj0)*(py)*(sj1)*(sj3)))+(((0.0495054651867961)*(cj1)*(cj3)*(pp)))+(((-0.00137784309743520)*(cj0)*(cj3)*(py)*(sj1)))+(((0.00620928554381296)*(cj1)))+(((0.000402192976073164)*(cj1)*((cj3)*(cj3))))+(((0.00137784309743520)*(cj3)*(px)*(sj0)*(sj1)))+(((-0.00137262137532159)*(py)*(sj0)*(sj1)))+(((-0.0495054651867961)*(pp)*(sj1)*(sj3)))+(((-0.00781407233391970)*(cj0)*(cj1)*(px)*(sj3)))+(((0.000567375750897680)*(px)*(sj0)*(sj1)*(sj3)))+(((-0.00866660996581944)*(cj1)*(py)*(sj0)))+(((-0.00781407233391970)*(cj1)*(py)*(sj0)*(sj3)))+(((-0.00321772135468784)*(cj0)*(px)*(sj1)*(sj3)))+(((0.00169066690488961)*(sj1)*((sj3)*(sj3))))+(((-0.00574553103162107)*(cj1)*(cj3)))+(((-0.00152816971858993)*(cj0)*(cj1)*(py)))+(((0.00959143594226369)*(cj1)*(sj3)))+(((-0.000242032170494404)*(cj0)*(py)*(sj1)))))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IKReal evalcond[5];
IKReal x76=IKsin(j2);
IKReal x77=IKcos(j2);
evalcond[0]=((0.0580650000000000)+(((-0.0649972956025481)*(py)*(sj0)))+(((-0.0346952602909277)*(x77)))+(((0.0114608710120556)*(px)*(sj0)))+(((-0.0296593095156779)*(sj3)*(x76)))+(((-0.0234021668514856)*(x76)))+(((0.0296593095156779)*(cj3)*(x77)))+(((-0.0649972956025481)*(cj0)*(px)))+(((-0.00812421365107049)*(cj3)))+(((-0.0114608710120556)*(cj0)*(py)))+(((0.0341511164173554)*(sj3)))+(((-1.00000000000000)*(pp)))+(((-0.0272910197510686)*(sj3)*(x77)))+(((-0.0272910197510686)*(cj3)*(x76))));
evalcond[1]=((((0.0495054651867961)*(sj1)*(sj3)*(x76)))+(((0.120221499394405)*(cj1)*(sj3)*(x76)))+(((-0.0495054651867961)*(cj3)*(sj1)*(x77)))+(((-0.0495054651867961)*(cj1)*(sj3)*(x77)))+(((0.133338008689083)*(cj1)*(x76)))+(((0.0211181305713861)*(sj1)*(x76)))+(((-0.0495054651867961)*(cj1)*(cj3)*(x76)))+(((-0.0211181305713861)*(cj1)*(x77)))+(((0.133338008689083)*(sj1)*(x77)))+(((0.120221499394405)*(cj3)*(sj1)*(x76)))+(((-0.140477556470912)*(sj1)))+(((-1.00000000000000)*(pz)))+(((0.120221499394405)*(sj1)*(sj3)*(x77)))+(((-0.0655061533595257)*(cj1)))+(((-0.120221499394405)*(cj1)*(cj3)*(x77))));
evalcond[2]=((((0.416199688533230)*(cj1)*(py)*(sj0)))+(((0.906306815941368)*(cj1)*(pz)))+(((-0.111920194486864)*(x76)))+(((0.157379780528043)*(cj0)*(py)*(sj1)))+(((0.892537757914323)*(py)*(sj0)*(sj1)))+(((0.0754908608112439)*(x77)))+(((-0.157379780528043)*(px)*(sj0)*(sj1)))+(((-0.422620344255005)*(pz)*(sj1)))+(((0.892537757914323)*(cj0)*(px)*(sj1)))+(((0.0733878371602598)*(cj0)*(cj1)*(py)))+(((0.416199688533230)*(cj0)*(cj1)*(px)))+(((0.0956751919860577)*(cj3)*(x76)))+(((-0.0733878371602598)*(cj1)*(px)*(sj0)))+(((0.0956751919860577)*(sj3)*(x77)))+(((0.0139464713604152)*(cj1)))+(((0.0299081249260652)*(sj1)))+(((-0.0880355475840924)*(sj3)*(x76)))+(((0.0880355475840924)*(cj3)*(x77))));
evalcond[3]=((-0.155000000000000)+(((0.0733878371602598)*(px)*(sj0)*(sj1)))+(((-0.0139464713604152)*(sj1)))+(((-0.0733878371602598)*(cj0)*(py)*(sj1)))+(((0.0299081249260652)*(cj1)))+(((-0.0956751919860577)*(cj3)*(x77)))+(((-0.906306815941368)*(pz)*(sj1)))+(((0.0880355475840924)*(sj3)*(x77)))+(((0.0754908608112439)*(x76)))+(((0.111920194486864)*(x77)))+(((0.157379780528043)*(cj0)*(cj1)*(py)))+(((-0.157379780528043)*(cj1)*(px)*(sj0)))+(((-0.422620344255005)*(cj1)*(pz)))+(((0.892537757914323)*(cj0)*(cj1)*(px)))+(((0.0956751919860577)*(sj3)*(x76)))+(((0.892537757914323)*(cj1)*(py)*(sj0)))+(((-0.416199688533230)*(cj0)*(px)*(sj1)))+(((-0.416199688533230)*(py)*(sj0)*(sj1)))+(((0.0880355475840924)*(cj3)*(x76))));
evalcond[4]=((0.0330000000000000)+(((0.0495054651867961)*(cj3)*(sj1)*(x76)))+(((0.0495054651867961)*(sj1)*(sj3)*(x77)))+(((0.120221499394405)*(cj1)*(sj3)*(x77)))+(((0.0495054651867961)*(cj1)*(sj3)*(x76)))+(((0.133338008689083)*(cj1)*(x77)))+(((0.0211181305713861)*(sj1)*(x77)))+(((-0.140477556470912)*(cj1)))+(((0.0655061533595257)*(sj1)))+(((-0.0495054651867961)*(cj1)*(cj3)*(x77)))+(((-0.133338008689083)*(sj1)*(x76)))+(((0.984807509129517)*(py)*(sj0)))+(((0.0211181305713861)*(cj1)*(x76)))+(((-0.120221499394405)*(sj1)*(sj3)*(x76)))+(((0.120221499394405)*(cj1)*(cj3)*(x76)))+(((-0.173649560788721)*(px)*(sj0)))+(((0.120221499394405)*(cj3)*(sj1)*(x77)))+(((0.984807509129517)*(cj0)*(px)))+(((0.173649560788721)*(cj0)*(py))));
if( IKabs(evalcond[0]) > 0.000001  || IKabs(evalcond[1]) > 0.000001  || IKabs(evalcond[2]) > 0.000001  || IKabs(evalcond[3]) > 0.000001  || IKabs(evalcond[4]) > 0.000001  )
{
continue;
}
}

{
vsolutions.push_back(IKSolution()); IKSolution& solution = vsolutions.back();
solution.basesol.resize(5);
solution.basesol[0].foffset = j0;
solution.basesol[0].indices[0] = _ij0[0];
solution.basesol[0].indices[1] = _ij0[1];
solution.basesol[0].maxsolutions = _nj0;
solution.basesol[1].foffset = j1;
solution.basesol[1].indices[0] = _ij1[0];
solution.basesol[1].indices[1] = _ij1[1];
solution.basesol[1].maxsolutions = _nj1;
solution.basesol[2].foffset = j2;
solution.basesol[2].indices[0] = _ij2[0];
solution.basesol[2].indices[1] = _ij2[1];
solution.basesol[2].maxsolutions = _nj2;
solution.basesol[3].foffset = j3;
solution.basesol[3].indices[0] = _ij3[0];
solution.basesol[3].indices[1] = _ij3[1];
solution.basesol[3].maxsolutions = _nj3;
solution.basesol[4].foffset = j4;
solution.basesol[4].indices[0] = _ij4[0];
solution.basesol[4].indices[1] = _ij4[1];
solution.basesol[4].maxsolutions = _nj4;
solution.vfree.resize(0);
}
}
}

}

}
}
}

}

}

} else
{
{
IKReal j2array[2], cj2array[2], sj2array[2];
bool j2valid[2]={false};
_nj2 = 2;
IKReal x78=((0.0272910197510686)*(cj3));
IKReal x79=((0.0296593095156779)*(sj3));
IKReal x80=((0.0272910197510686)*(sj3));
IKReal x81=((0.0296593095156779)*(cj3));
IKReal x82=((0.0234021668514856)+(x79)+(x78));
IKReal x83=(x82)*(x82);
IKReal x84=((0.0346952602909277)+(x80));
IKReal x85=((x84)+(((-1.00000000000000)*(x81))));
IKReal x86=(x85)*(x85);
IKReal x87=((x83)+(x86));
if( (x87) < (IKReal)-0.00001 )
    continue;
IKReal x88=IKsqrt(x87);
IKReal x89=IKabs(x88);
IKReal x90=((IKabs(x89) != 0)?((IKReal)1/(x89)):(IKReal)1.0e30);
IKReal x91=((0.0114608710120556)*(px)*(sj0));
IKReal x92=((0.0341511164173554)*(sj3));
IKReal x93=((0.0580650000000000)+(x91)+(x92));
IKReal x94=((0.00812421365107049)*(cj3));
IKReal x95=((0.0114608710120556)*(cj0)*(py));
IKReal x96=((0.0649972956025481)*(py)*(sj0));
IKReal x97=((0.0649972956025481)*(cj0)*(px));
IKReal x98=((pp)+(x95)+(x94)+(x97)+(x96));
IKReal x99=((x93)+(((-1.00000000000000)*(x98))));
IKReal x100=((x90)*(x99));
if( (x100) < -1-IKFAST_SINCOS_THRESH || (x100) > 1+IKFAST_SINCOS_THRESH )
    continue;
IKReal x101=IKasin(x100);
IKReal x102=((-0.0346952602909277)+(x81));
IKReal x103=((x102)+(((-1.00000000000000)*(x80))));
IKReal x104=((-0.0234021668514856)+(x82));
IKReal x105=((-0.0234021668514856)+(((-1.00000000000000)*(x104))));
if( IKabs(x103) < IKFAST_ATAN2_MAGTHRESH && IKabs(x105) < IKFAST_ATAN2_MAGTHRESH )
    continue;
IKReal x106=IKatan2(x103, x105);
j2array[0]=((((-1.00000000000000)*(x101)))+(((-1.00000000000000)*(x106))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
j2array[1]=((3.14159265358979)+(((-1.00000000000000)*(x106)))+(x101));
sj2array[1]=IKsin(j2array[1]);
cj2array[1]=IKcos(j2array[1]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
if( j2array[1] > IKPI )
{
    j2array[1]-=IK2PI;
}
else if( j2array[1] < -IKPI )
{    j2array[1]+=IK2PI;
}
j2valid[1] = true;
for(int ij2 = 0; ij2 < 2; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 2; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];

{
IKReal dummyeval[1];
IKReal gconst1;
gconst1=IKsign(((((0.0166139550667555)*(cj3)*(px)*(sj0)*(sj2)))+(((-0.0152873341717722)*(px)*(sj0)*(sj2)*(sj3)))+(((-0.111920194486864)*(cj2)*(pz)))+(((-0.0131089548234350)*(cj0)*(cj2)*(py)))+(((0.110219847953899)*(cj0)*(px)*(sj2)))+(((-0.0942216475052779)*(cj0)*(cj3)*(px)*(sj2)))+(((-0.0743439665975642)*(cj0)*(cj2)*(px)))+(((0.0131089548234350)*(cj2)*(px)*(sj0)))+(((-0.0880355475840924)*(cj2)*(pz)*(sj3)))+(((-0.0743439665975642)*(cj2)*(py)*(sj0)))+(((0.0866980683311431)*(cj0)*(px)*(sj2)*(sj3)))+(((-0.0866980683311431)*(cj0)*(cj2)*(cj3)*(px)))+(((-0.0166139550667555)*(cj0)*(cj2)*(py)*(sj3)))+(((-0.00290517307027505)*(cj2)*(cj3)))+(((-0.00315728133553991)*(cj2)*(sj3)))+(((-0.0866980683311431)*(cj2)*(cj3)*(py)*(sj0)))+(((-0.0942216475052779)*(cj0)*(cj2)*(px)*(sj3)))+(((-0.0880355475840924)*(cj3)*(pz)*(sj2)))+(((0.0194348926160322)*(cj0)*(py)*(sj2)))+(((0.0956751919860577)*(cj2)*(cj3)*(pz)))+(((-0.0166139550667555)*(cj0)*(cj3)*(py)*(sj2)))+(((-0.0942216475052779)*(cj3)*(py)*(sj0)*(sj2)))+(((-0.00249119840677105)*(cj2)))+(((-0.00315728133553991)*(cj3)*(sj2)))+(((0.0152873341717722)*(cj0)*(py)*(sj2)*(sj3)))+(((-0.0956751919860577)*(pz)*(sj2)*(sj3)))+(((-0.0152873341717722)*(cj0)*(cj2)*(cj3)*(py)))+(((-0.0194348926160322)*(px)*(sj0)*(sj2)))+(((-0.0942216475052779)*(cj2)*(py)*(sj0)*(sj3)))+(((0.0152873341717722)*(cj2)*(cj3)*(px)*(sj0)))+(((0.110219847953899)*(py)*(sj0)*(sj2)))+(((-0.0754908608112440)*(pz)*(sj2)))+(((0.0166139550667555)*(cj2)*(px)*(sj0)*(sj3)))+(((0.0866980683311431)*(py)*(sj0)*(sj2)*(sj3)))+(((0.00369336641806650)*(sj2)))+(((0.155000000000000)*(pz)))+(((0.00290517307027505)*(sj2)*(sj3)))));
dummyeval[0]=((((-6.13653819391559)*(cj0)*(cj2)*(cj3)*(py)))+(((34.8017516772244)*(cj0)*(px)*(sj2)*(sj3)))+(((1.16617490697602)*(sj2)*(sj3)))+(((-7.80142302724999)*(px)*(sj0)*(sj2)))+(((-30.3030303030303)*(pz)*(sj2)))+(((-6.66906137287137)*(cj0)*(cj2)*(py)*(sj3)))+(((5.26210790268853)*(cj2)*(px)*(sj0)))+(((1.48256614488351)*(sj2)))+(((-6.66906137287137)*(cj0)*(cj3)*(py)*(sj2)))+(((-1.26737450014357)*(cj3)*(sj2)))+(((-37.8218158975955)*(cj3)*(py)*(sj0)*(sj2)))+(((-37.8218158975955)*(cj2)*(py)*(sj0)*(sj3)))+(((-29.8426517918036)*(cj2)*(py)*(sj0)))+(((-35.3386335447280)*(cj2)*(pz)*(sj3)))+(((34.8017516772244)*(py)*(sj0)*(sj2)*(sj3)))+(((62.2190507101770)*(pz)))+(((6.66906137287137)*(cj2)*(px)*(sj0)*(sj3)))+(((6.13653819391559)*(cj0)*(py)*(sj2)*(sj3)))+(((-44.9262468146519)*(cj2)*(pz)))+(((6.66906137287137)*(cj3)*(px)*(sj0)*(sj2)))+(((-6.13653819391559)*(px)*(sj0)*(sj2)*(sj3)))+(((44.2437052200752)*(cj0)*(px)*(sj2)))+(((-29.8426517918036)*(cj0)*(cj2)*(px)))+(((-1.00000000000000)*(cj2)))+(((-38.4052878831383)*(pz)*(sj2)*(sj3)))+(((7.80142302724999)*(cj0)*(py)*(sj2)))+(((-1.16617490697602)*(cj2)*(cj3)))+(((-5.26210790268853)*(cj0)*(cj2)*(py)))+(((-1.26737450014357)*(cj2)*(sj3)))+(((-35.3386335447280)*(cj3)*(pz)*(sj2)))+(((6.13653819391559)*(cj2)*(cj3)*(px)*(sj0)))+(((-37.8218158975955)*(cj0)*(cj3)*(px)*(sj2)))+(((38.4052878831383)*(cj2)*(cj3)*(pz)))+(((-34.8017516772244)*(cj2)*(cj3)*(py)*(sj0)))+(((44.2437052200752)*(py)*(sj0)*(sj2)))+(((-37.8218158975955)*(cj0)*(cj2)*(px)*(sj3)))+(((-34.8017516772244)*(cj0)*(cj2)*(cj3)*(px))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
continue;

} else
{
{
IKReal j1array[1], cj1array[1], sj1array[1];
bool j1valid[1]={false};
_nj1 = 1;
IKReal x107=(cj2)*(cj2);
IKReal x108=(cj3)*(cj3);
IKReal x109=(sj2)*(sj2);
IKReal x110=(sj3)*(sj3);
IKReal x111=(pz)*(pz);
if( IKabs(((gconst1)*(((((0.00473644488610582)*(x107)*(x110)))+(((0.0149232158649733)*(x109)))+(((-0.0158604557715362)*(cj2)*(sj2)*(x110)))+(((0.00575769137861631)*(sj3)*(x107)))+(((0.0733878371602598)*(px)*(pz)*(sj0)))+(((0.0158604557715362)*(cj2)*(sj2)*(x108)))+(((-0.0116946412889225)*(cj2)*(cj3)*(sj2)*(sj3)))+(((-0.0292325715383419)*(cj2)*(sj2)*(sj3)))+(((0.0158604557715362)*(cj3)*(sj3)*(x107)))+(((-0.00576687008113338)*(sj2)*(sj3)))+(((-0.0158604557715362)*(cj3)*(sj3)*(x109)))+(((-0.0194360068238237)*(cj2)*(cj3)*(sj2)))+(((0.00494511590554396)*(cj2)))+(((-0.0182978008722355)*(cj3)*(x109)))+(((0.0105837655305671)*(x109)*(x110)))+(((-0.416199688533230)*(cj0)*(px)*(pz)))+(((0.0251936982024400)*(sj3)*(x109)))+(((-0.0139464713604152)*(pz)))+(((-0.906306815941368)*(x111)))+(((0.0109347706661064)*(cj3)*(x107)))+(((-0.0124293463355445)*(cj2)*(sj2)))+(((0.00159422585555818)*(x107)))+(((-0.416199688533230)*(py)*(pz)*(sj0)))+(((0.00473644488610582)*(x108)*(x109)))+(((-0.0733878371602598)*(cj0)*(py)*(pz)))+(((0.00626731379894077)*(cj3)*(sj2)))+(((0.00576687008113338)*(cj2)*(cj3)))+(((0.0105837655305671)*(x107)*(x108)))+(((-0.00733146142408444)*(sj2)))+(((0.00626731379894077)*(cj2)*(sj3))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((gconst1)*(((((-0.0106047716626493)*(cj2)))+(((-0.00435824073612482)*(x107)*(x108)))+(((0.0115022150354114)*(x107)*(x110)))+(((0.0100658010547960)*(x107)))+(((-0.00435824073612482)*(x109)*(x110)))+(((0.892537757914323)*(cj0)*(px)*(pz)))+(((-0.0114347323969202)*(cj3)*(x109)))+(((0.00584732064446125)*(cj2)*(sj2)*(x108)))+(((-0.0134402171850868)*(cj3)*(sj2)))+(((0.0218327640576687)*(sj3)*(x107)))+(((0.0157222754412639)*(sj2)))+(((0.0292325715383419)*(cj2)*(cj3)*(sj2)))+(((-0.0194360068238237)*(cj2)*(sj2)*(sj3)))+(((0.0317209115430725)*(cj2)*(cj3)*(sj2)*(sj3)))+(((-0.157379780528043)*(px)*(pz)*(sj0)))+(((0.0299081249260652)*(pz)))+(((0.00584732064446125)*(cj3)*(sj3)*(x107)))+(((-0.422620344255005)*(x111)))+(((-0.00739980748067322)*(sj3)*(x109)))+(((-0.00236354528074851)*(x109)))+(((0.892537757914323)*(py)*(pz)*(sj0)))+(((0.157379780528043)*(cj0)*(py)*(pz)))+(((-0.00584732064446125)*(cj3)*(sj3)*(x109)))+(((-0.0133289900094151)*(cj2)*(sj2)))+(((-0.00584732064446125)*(cj2)*(sj2)*(x110)))+(((0.0123670186071920)*(sj2)*(sj3)))+(((-0.0123670186071920)*(cj2)*(cj3)))+(((0.0115022150354114)*(x108)*(x109)))+(((0.00800127442690355)*(cj3)*(x107)))+(((-0.0134402171850868)*(cj2)*(sj3))))))) < IKFAST_ATAN2_MAGTHRESH )
    continue;
j1array[0]=IKatan2(((gconst1)*(((((0.00473644488610582)*(x107)*(x110)))+(((0.0149232158649733)*(x109)))+(((-0.0158604557715362)*(cj2)*(sj2)*(x110)))+(((0.00575769137861631)*(sj3)*(x107)))+(((0.0733878371602598)*(px)*(pz)*(sj0)))+(((0.0158604557715362)*(cj2)*(sj2)*(x108)))+(((-0.0116946412889225)*(cj2)*(cj3)*(sj2)*(sj3)))+(((-0.0292325715383419)*(cj2)*(sj2)*(sj3)))+(((0.0158604557715362)*(cj3)*(sj3)*(x107)))+(((-0.00576687008113338)*(sj2)*(sj3)))+(((-0.0158604557715362)*(cj3)*(sj3)*(x109)))+(((-0.0194360068238237)*(cj2)*(cj3)*(sj2)))+(((0.00494511590554396)*(cj2)))+(((-0.0182978008722355)*(cj3)*(x109)))+(((0.0105837655305671)*(x109)*(x110)))+(((-0.416199688533230)*(cj0)*(px)*(pz)))+(((0.0251936982024400)*(sj3)*(x109)))+(((-0.0139464713604152)*(pz)))+(((-0.906306815941368)*(x111)))+(((0.0109347706661064)*(cj3)*(x107)))+(((-0.0124293463355445)*(cj2)*(sj2)))+(((0.00159422585555818)*(x107)))+(((-0.416199688533230)*(py)*(pz)*(sj0)))+(((0.00473644488610582)*(x108)*(x109)))+(((-0.0733878371602598)*(cj0)*(py)*(pz)))+(((0.00626731379894077)*(cj3)*(sj2)))+(((0.00576687008113338)*(cj2)*(cj3)))+(((0.0105837655305671)*(x107)*(x108)))+(((-0.00733146142408444)*(sj2)))+(((0.00626731379894077)*(cj2)*(sj3)))))), ((gconst1)*(((((-0.0106047716626493)*(cj2)))+(((-0.00435824073612482)*(x107)*(x108)))+(((0.0115022150354114)*(x107)*(x110)))+(((0.0100658010547960)*(x107)))+(((-0.00435824073612482)*(x109)*(x110)))+(((0.892537757914323)*(cj0)*(px)*(pz)))+(((-0.0114347323969202)*(cj3)*(x109)))+(((0.00584732064446125)*(cj2)*(sj2)*(x108)))+(((-0.0134402171850868)*(cj3)*(sj2)))+(((0.0218327640576687)*(sj3)*(x107)))+(((0.0157222754412639)*(sj2)))+(((0.0292325715383419)*(cj2)*(cj3)*(sj2)))+(((-0.0194360068238237)*(cj2)*(sj2)*(sj3)))+(((0.0317209115430725)*(cj2)*(cj3)*(sj2)*(sj3)))+(((-0.157379780528043)*(px)*(pz)*(sj0)))+(((0.0299081249260652)*(pz)))+(((0.00584732064446125)*(cj3)*(sj3)*(x107)))+(((-0.422620344255005)*(x111)))+(((-0.00739980748067322)*(sj3)*(x109)))+(((-0.00236354528074851)*(x109)))+(((0.892537757914323)*(py)*(pz)*(sj0)))+(((0.157379780528043)*(cj0)*(py)*(pz)))+(((-0.00584732064446125)*(cj3)*(sj3)*(x109)))+(((-0.0133289900094151)*(cj2)*(sj2)))+(((-0.00584732064446125)*(cj2)*(sj2)*(x110)))+(((0.0123670186071920)*(sj2)*(sj3)))+(((-0.0123670186071920)*(cj2)*(cj3)))+(((0.0115022150354114)*(x108)*(x109)))+(((0.00800127442690355)*(cj3)*(x107)))+(((-0.0134402171850868)*(cj2)*(sj3)))))));
sj1array[0]=IKsin(j1array[0]);
cj1array[0]=IKcos(j1array[0]);
if( j1array[0] > IKPI )
{
    j1array[0]-=IK2PI;
}
else if( j1array[0] < -IKPI )
{    j1array[0]+=IK2PI;
}
j1valid[0] = true;
for(int ij1 = 0; ij1 < 1; ++ij1)
{
if( !j1valid[ij1] )
{
    continue;
}
_ij1[0] = ij1; _ij1[1] = -1;
for(int iij1 = ij1+1; iij1 < 1; ++iij1)
{
if( j1valid[iij1] && IKabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && IKabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
{
    j1valid[iij1]=false; _ij1[1] = iij1; break; 
}
}
j1 = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
{
IKReal evalcond[5];
IKReal x112=IKsin(j1);
IKReal x113=IKcos(j1);
evalcond[0]=((((-0.140477556470912)*(x112)))+(((-0.0495054651867961)*(cj2)*(cj3)*(x112)))+(((-0.0655061533595257)*(x113)))+(((-0.0495054651867961)*(cj3)*(sj2)*(x113)))+(((-0.120221499394405)*(cj2)*(cj3)*(x113)))+(((-0.0211181305713861)*(cj2)*(x113)))+(((0.0495054651867961)*(sj2)*(sj3)*(x112)))+(((0.0211181305713861)*(sj2)*(x112)))+(((0.120221499394405)*(cj3)*(sj2)*(x112)))+(((0.133338008689083)*(sj2)*(x113)))+(((-0.0495054651867961)*(cj2)*(sj3)*(x113)))+(((0.120221499394405)*(sj2)*(sj3)*(x113)))+(((-1.00000000000000)*(pz)))+(((0.133338008689083)*(cj2)*(x112)))+(((0.120221499394405)*(cj2)*(sj3)*(x112))));
evalcond[1]=((0.0100150000000000)+(((0.0114608710120556)*(px)*(sj0)))+(((0.0227502295196805)*(px)*(sj0)*(x112)))+(((0.0487877319636934)*(cj0)*(py)*(x113)))+(((0.276686704953440)*(cj0)*(px)*(x113)))+(((-0.00812421365107049)*(cj3)))+(((0.276686704953440)*(py)*(sj0)*(x113)))+(((-0.00432340612172870)*(x112)))+(((-0.0649972956025481)*(py)*(sj0)))+(((-0.129021903445301)*(cj0)*(px)*(x112)))+(((-0.131012306719051)*(pz)*(x113)))+(((-0.0227502295196805)*(cj0)*(py)*(x112)))+(((-0.0114608710120556)*(cj0)*(py)))+(((-0.280955112941824)*(pz)*(x112)))+(((-0.0487877319636934)*(px)*(sj0)*(x113)))+(((0.0341511164173554)*(sj3)))+(((-1.00000000000000)*(pp)))+(((-0.0649972956025481)*(cj0)*(px)))+(((-0.129021903445301)*(py)*(sj0)*(x112)))+(((0.00927151872708020)*(x113))));
evalcond[2]=((((0.0956751919860577)*(cj3)*(sj2)))+(((0.0956751919860577)*(cj2)*(sj3)))+(((0.906306815941368)*(pz)*(x113)))+(((-0.157379780528043)*(px)*(sj0)*(x112)))+(((-0.0880355475840924)*(sj2)*(sj3)))+(((-0.422620344255005)*(pz)*(x112)))+(((0.892537757914323)*(cj0)*(px)*(x112)))+(((0.0733878371602598)*(cj0)*(py)*(x113)))+(((0.892537757914323)*(py)*(sj0)*(x112)))+(((0.0880355475840924)*(cj2)*(cj3)))+(((0.0299081249260652)*(x112)))+(((0.157379780528043)*(cj0)*(py)*(x112)))+(((-0.111920194486864)*(sj2)))+(((0.0754908608112439)*(cj2)))+(((-0.0733878371602598)*(px)*(sj0)*(x113)))+(((0.0139464713604152)*(x113)))+(((0.416199688533230)*(cj0)*(px)*(x113)))+(((0.416199688533230)*(py)*(sj0)*(x113))));
evalcond[3]=((-0.155000000000000)+(((-0.416199688533230)*(cj0)*(px)*(x112)))+(((0.0956751919860577)*(sj2)*(sj3)))+(((0.0880355475840924)*(cj3)*(sj2)))+(((-0.906306815941368)*(pz)*(x112)))+(((-0.157379780528043)*(px)*(sj0)*(x113)))+(((-0.416199688533230)*(py)*(sj0)*(x112)))+(((-0.0956751919860577)*(cj2)*(cj3)))+(((-0.0733878371602598)*(cj0)*(py)*(x112)))+(((-0.422620344255005)*(pz)*(x113)))+(((0.0754908608112439)*(sj2)))+(((0.892537757914323)*(cj0)*(px)*(x113)))+(((0.111920194486864)*(cj2)))+(((0.892537757914323)*(py)*(sj0)*(x113)))+(((0.0733878371602598)*(px)*(sj0)*(x112)))+(((0.0880355475840924)*(cj2)*(sj3)))+(((-0.0139464713604152)*(x112)))+(((0.0299081249260652)*(x113)))+(((0.157379780528043)*(cj0)*(py)*(x113))));
evalcond[4]=((0.0330000000000000)+(((-0.140477556470912)*(x113)))+(((-0.0495054651867961)*(cj2)*(cj3)*(x113)))+(((0.0655061533595257)*(x112)))+(((0.0211181305713861)*(cj2)*(x112)))+(((0.0495054651867961)*(cj3)*(sj2)*(x112)))+(((0.984807509129517)*(py)*(sj0)))+(((0.0495054651867961)*(sj2)*(sj3)*(x113)))+(((-0.173649560788721)*(px)*(sj0)))+(((-0.120221499394405)*(sj2)*(sj3)*(x112)))+(((0.0211181305713861)*(sj2)*(x113)))+(((-0.133338008689083)*(sj2)*(x112)))+(((0.120221499394405)*(cj3)*(sj2)*(x113)))+(((0.984807509129517)*(cj0)*(px)))+(((0.173649560788721)*(cj0)*(py)))+(((0.133338008689083)*(cj2)*(x113)))+(((0.120221499394405)*(cj2)*(sj3)*(x113)))+(((0.0495054651867961)*(cj2)*(sj3)*(x112)))+(((0.120221499394405)*(cj2)*(cj3)*(x112))));
if( IKabs(evalcond[0]) > 0.000001  || IKabs(evalcond[1]) > 0.000001  || IKabs(evalcond[2]) > 0.000001  || IKabs(evalcond[3]) > 0.000001  || IKabs(evalcond[4]) > 0.000001  )
{
continue;
}
}

{
vsolutions.push_back(IKSolution()); IKSolution& solution = vsolutions.back();
solution.basesol.resize(5);
solution.basesol[0].foffset = j0;
solution.basesol[0].indices[0] = _ij0[0];
solution.basesol[0].indices[1] = _ij0[1];
solution.basesol[0].maxsolutions = _nj0;
solution.basesol[1].foffset = j1;
solution.basesol[1].indices[0] = _ij1[0];
solution.basesol[1].indices[1] = _ij1[1];
solution.basesol[1].maxsolutions = _nj1;
solution.basesol[2].foffset = j2;
solution.basesol[2].indices[0] = _ij2[0];
solution.basesol[2].indices[1] = _ij2[1];
solution.basesol[2].maxsolutions = _nj2;
solution.basesol[3].foffset = j3;
solution.basesol[3].indices[0] = _ij3[0];
solution.basesol[3].indices[1] = _ij3[1];
solution.basesol[3].maxsolutions = _nj3;
solution.basesol[4].foffset = j4;
solution.basesol[4].indices[0] = _ij4[0];
solution.basesol[4].indices[1] = _ij4[1];
solution.basesol[4].maxsolutions = _nj4;
solution.vfree.resize(0);
}
}
}

}

}
}
}

}

}

} else
{
{
IKReal j2array[2], cj2array[2], sj2array[2];
bool j2valid[2]={false};
_nj2 = 2;
IKReal x114=((0.0272910197510686)*(cj3));
IKReal x115=((0.0296593095156779)*(sj3));
IKReal x116=((0.0272910197510686)*(sj3));
IKReal x117=((0.0296593095156779)*(cj3));
IKReal x118=((0.0234021668514856)+(x115)+(x114));
IKReal x119=(x118)*(x118);
IKReal x120=((0.0346952602909277)+(x116));
IKReal x121=((((-1.00000000000000)*(x117)))+(x120));
IKReal x122=(x121)*(x121);
IKReal x123=((x122)+(x119));
if( (x123) < (IKReal)-0.00001 )
    continue;
IKReal x124=IKsqrt(x123);
IKReal x125=IKabs(x124);
IKReal x126=((IKabs(x125) != 0)?((IKReal)1/(x125)):(IKReal)1.0e30);
IKReal x127=((0.0114608710120556)*(px)*(sj0));
IKReal x128=((0.0341511164173554)*(sj3));
IKReal x129=((0.0580650000000000)+(x127)+(x128));
IKReal x130=((0.00812421365107049)*(cj3));
IKReal x131=((0.0114608710120556)*(cj0)*(py));
IKReal x132=((0.0649972956025481)*(py)*(sj0));
IKReal x133=((0.0649972956025481)*(cj0)*(px));
IKReal x134=((x131)+(x130)+(x133)+(x132)+(pp));
IKReal x135=((x129)+(((-1.00000000000000)*(x134))));
IKReal x136=((x126)*(x135));
if( (x136) < -1-IKFAST_SINCOS_THRESH || (x136) > 1+IKFAST_SINCOS_THRESH )
    continue;
IKReal x137=IKasin(x136);
IKReal x138=((-0.0346952602909277)+(x117));
IKReal x139=((((-1.00000000000000)*(x116)))+(x138));
IKReal x140=((-0.0234021668514856)+(x118));
IKReal x141=((-0.0234021668514856)+(((-1.00000000000000)*(x140))));
if( IKabs(x139) < IKFAST_ATAN2_MAGTHRESH && IKabs(x141) < IKFAST_ATAN2_MAGTHRESH )
    continue;
IKReal x142=IKatan2(x139, x141);
j2array[0]=((((-1.00000000000000)*(x142)))+(((-1.00000000000000)*(x137))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
j2array[1]=((3.14159265358979)+(((-1.00000000000000)*(x142)))+(x137));
sj2array[1]=IKsin(j2array[1]);
cj2array[1]=IKcos(j2array[1]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
if( j2array[1] > IKPI )
{
    j2array[1]-=IK2PI;
}
else if( j2array[1] < -IKPI )
{    j2array[1]+=IK2PI;
}
j2valid[1] = true;
for(int ij2 = 0; ij2 < 2; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 2; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];

{
IKReal dummyeval[1];
IKReal gconst1;
gconst1=IKsign(((((0.0166139550667555)*(cj3)*(px)*(sj0)*(sj2)))+(((-0.0152873341717722)*(px)*(sj0)*(sj2)*(sj3)))+(((-0.111920194486864)*(cj2)*(pz)))+(((-0.0131089548234350)*(cj0)*(cj2)*(py)))+(((0.110219847953899)*(cj0)*(px)*(sj2)))+(((-0.0942216475052779)*(cj0)*(cj3)*(px)*(sj2)))+(((-0.0743439665975642)*(cj0)*(cj2)*(px)))+(((0.0131089548234350)*(cj2)*(px)*(sj0)))+(((-0.0880355475840924)*(cj2)*(pz)*(sj3)))+(((-0.0743439665975642)*(cj2)*(py)*(sj0)))+(((0.0866980683311431)*(cj0)*(px)*(sj2)*(sj3)))+(((-0.0866980683311431)*(cj0)*(cj2)*(cj3)*(px)))+(((-0.0166139550667555)*(cj0)*(cj2)*(py)*(sj3)))+(((-0.00290517307027505)*(cj2)*(cj3)))+(((-0.00315728133553991)*(cj2)*(sj3)))+(((-0.0866980683311431)*(cj2)*(cj3)*(py)*(sj0)))+(((-0.0942216475052779)*(cj0)*(cj2)*(px)*(sj3)))+(((-0.0880355475840924)*(cj3)*(pz)*(sj2)))+(((0.0194348926160322)*(cj0)*(py)*(sj2)))+(((0.0956751919860577)*(cj2)*(cj3)*(pz)))+(((-0.0166139550667555)*(cj0)*(cj3)*(py)*(sj2)))+(((-0.0942216475052779)*(cj3)*(py)*(sj0)*(sj2)))+(((-0.00249119840677105)*(cj2)))+(((-0.00315728133553991)*(cj3)*(sj2)))+(((0.0152873341717722)*(cj0)*(py)*(sj2)*(sj3)))+(((-0.0956751919860577)*(pz)*(sj2)*(sj3)))+(((-0.0152873341717722)*(cj0)*(cj2)*(cj3)*(py)))+(((-0.0194348926160322)*(px)*(sj0)*(sj2)))+(((-0.0942216475052779)*(cj2)*(py)*(sj0)*(sj3)))+(((0.0152873341717722)*(cj2)*(cj3)*(px)*(sj0)))+(((0.110219847953899)*(py)*(sj0)*(sj2)))+(((-0.0754908608112440)*(pz)*(sj2)))+(((0.0166139550667555)*(cj2)*(px)*(sj0)*(sj3)))+(((0.0866980683311431)*(py)*(sj0)*(sj2)*(sj3)))+(((0.00369336641806650)*(sj2)))+(((0.155000000000000)*(pz)))+(((0.00290517307027505)*(sj2)*(sj3)))));
dummyeval[0]=((((-6.13653819391559)*(cj0)*(cj2)*(cj3)*(py)))+(((34.8017516772244)*(cj0)*(px)*(sj2)*(sj3)))+(((1.16617490697602)*(sj2)*(sj3)))+(((-7.80142302724999)*(px)*(sj0)*(sj2)))+(((-30.3030303030303)*(pz)*(sj2)))+(((-6.66906137287137)*(cj0)*(cj2)*(py)*(sj3)))+(((5.26210790268853)*(cj2)*(px)*(sj0)))+(((1.48256614488351)*(sj2)))+(((-6.66906137287137)*(cj0)*(cj3)*(py)*(sj2)))+(((-1.26737450014357)*(cj3)*(sj2)))+(((-37.8218158975955)*(cj3)*(py)*(sj0)*(sj2)))+(((-37.8218158975955)*(cj2)*(py)*(sj0)*(sj3)))+(((-29.8426517918036)*(cj2)*(py)*(sj0)))+(((-35.3386335447280)*(cj2)*(pz)*(sj3)))+(((34.8017516772244)*(py)*(sj0)*(sj2)*(sj3)))+(((62.2190507101770)*(pz)))+(((6.66906137287137)*(cj2)*(px)*(sj0)*(sj3)))+(((6.13653819391559)*(cj0)*(py)*(sj2)*(sj3)))+(((-44.9262468146519)*(cj2)*(pz)))+(((6.66906137287137)*(cj3)*(px)*(sj0)*(sj2)))+(((-6.13653819391559)*(px)*(sj0)*(sj2)*(sj3)))+(((44.2437052200752)*(cj0)*(px)*(sj2)))+(((-29.8426517918036)*(cj0)*(cj2)*(px)))+(((-1.00000000000000)*(cj2)))+(((-38.4052878831383)*(pz)*(sj2)*(sj3)))+(((7.80142302724999)*(cj0)*(py)*(sj2)))+(((-1.16617490697602)*(cj2)*(cj3)))+(((-5.26210790268853)*(cj0)*(cj2)*(py)))+(((-1.26737450014357)*(cj2)*(sj3)))+(((-35.3386335447280)*(cj3)*(pz)*(sj2)))+(((6.13653819391559)*(cj2)*(cj3)*(px)*(sj0)))+(((-37.8218158975955)*(cj0)*(cj3)*(px)*(sj2)))+(((38.4052878831383)*(cj2)*(cj3)*(pz)))+(((-34.8017516772244)*(cj2)*(cj3)*(py)*(sj0)))+(((44.2437052200752)*(py)*(sj0)*(sj2)))+(((-37.8218158975955)*(cj0)*(cj2)*(px)*(sj3)))+(((-34.8017516772244)*(cj0)*(cj2)*(cj3)*(px))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
continue;

} else
{
{
IKReal j1array[1], cj1array[1], sj1array[1];
bool j1valid[1]={false};
_nj1 = 1;
IKReal x143=(cj2)*(cj2);
IKReal x144=(cj3)*(cj3);
IKReal x145=(sj2)*(sj2);
IKReal x146=(sj3)*(sj3);
IKReal x147=(pz)*(pz);
if( IKabs(((gconst1)*(((((0.00575769137861631)*(sj3)*(x143)))+(((0.00473644488610582)*(x144)*(x145)))+(((0.0105837655305671)*(x143)*(x144)))+(((0.0733878371602598)*(px)*(pz)*(sj0)))+(((-0.0116946412889225)*(cj2)*(cj3)*(sj2)*(sj3)))+(((0.0105837655305671)*(x145)*(x146)))+(((-0.0292325715383419)*(cj2)*(sj2)*(sj3)))+(((-0.00576687008113338)*(sj2)*(sj3)))+(((0.0109347706661064)*(cj3)*(x143)))+(((-0.906306815941368)*(x147)))+(((-0.0194360068238237)*(cj2)*(cj3)*(sj2)))+(((0.00159422585555818)*(x143)))+(((0.00494511590554396)*(cj2)))+(((-0.416199688533230)*(cj0)*(px)*(pz)))+(((-0.0158604557715362)*(cj2)*(sj2)*(x146)))+(((-0.0139464713604152)*(pz)))+(((0.00473644488610582)*(x143)*(x146)))+(((0.0158604557715362)*(cj2)*(sj2)*(x144)))+(((-0.0124293463355445)*(cj2)*(sj2)))+(((0.0158604557715362)*(cj3)*(sj3)*(x143)))+(((-0.0158604557715362)*(cj3)*(sj3)*(x145)))+(((-0.416199688533230)*(py)*(pz)*(sj0)))+(((-0.0182978008722355)*(cj3)*(x145)))+(((-0.0733878371602598)*(cj0)*(py)*(pz)))+(((0.00626731379894077)*(cj3)*(sj2)))+(((0.0251936982024400)*(sj3)*(x145)))+(((0.0149232158649733)*(x145)))+(((0.00576687008113338)*(cj2)*(cj3)))+(((-0.00733146142408444)*(sj2)))+(((0.00626731379894077)*(cj2)*(sj3))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((gconst1)*(((((-0.0106047716626493)*(cj2)))+(((0.0115022150354114)*(x144)*(x145)))+(((-0.422620344255005)*(x147)))+(((-0.00435824073612482)*(x145)*(x146)))+(((0.892537757914323)*(cj0)*(px)*(pz)))+(((0.00800127442690355)*(cj3)*(x143)))+(((0.0115022150354114)*(x143)*(x146)))+(((0.00584732064446125)*(cj3)*(sj3)*(x143)))+(((-0.0134402171850868)*(cj3)*(sj2)))+(((0.0100658010547960)*(x143)))+(((0.0157222754412639)*(sj2)))+(((0.0292325715383419)*(cj2)*(cj3)*(sj2)))+(((-0.00739980748067322)*(sj3)*(x145)))+(((-0.00584732064446125)*(cj2)*(sj2)*(x146)))+(((-0.0194360068238237)*(cj2)*(sj2)*(sj3)))+(((0.0317209115430725)*(cj2)*(cj3)*(sj2)*(sj3)))+(((-0.00236354528074851)*(x145)))+(((-0.157379780528043)*(px)*(pz)*(sj0)))+(((-0.00584732064446125)*(cj3)*(sj3)*(x145)))+(((0.0299081249260652)*(pz)))+(((0.0218327640576687)*(sj3)*(x143)))+(((0.892537757914323)*(py)*(pz)*(sj0)))+(((-0.00435824073612482)*(x143)*(x144)))+(((0.157379780528043)*(cj0)*(py)*(pz)))+(((-0.0133289900094151)*(cj2)*(sj2)))+(((0.0123670186071920)*(sj2)*(sj3)))+(((-0.0123670186071920)*(cj2)*(cj3)))+(((-0.0114347323969202)*(cj3)*(x145)))+(((-0.0134402171850868)*(cj2)*(sj3)))+(((0.00584732064446125)*(cj2)*(sj2)*(x144))))))) < IKFAST_ATAN2_MAGTHRESH )
    continue;
j1array[0]=IKatan2(((gconst1)*(((((0.00575769137861631)*(sj3)*(x143)))+(((0.00473644488610582)*(x144)*(x145)))+(((0.0105837655305671)*(x143)*(x144)))+(((0.0733878371602598)*(px)*(pz)*(sj0)))+(((-0.0116946412889225)*(cj2)*(cj3)*(sj2)*(sj3)))+(((0.0105837655305671)*(x145)*(x146)))+(((-0.0292325715383419)*(cj2)*(sj2)*(sj3)))+(((-0.00576687008113338)*(sj2)*(sj3)))+(((0.0109347706661064)*(cj3)*(x143)))+(((-0.906306815941368)*(x147)))+(((-0.0194360068238237)*(cj2)*(cj3)*(sj2)))+(((0.00159422585555818)*(x143)))+(((0.00494511590554396)*(cj2)))+(((-0.416199688533230)*(cj0)*(px)*(pz)))+(((-0.0158604557715362)*(cj2)*(sj2)*(x146)))+(((-0.0139464713604152)*(pz)))+(((0.00473644488610582)*(x143)*(x146)))+(((0.0158604557715362)*(cj2)*(sj2)*(x144)))+(((-0.0124293463355445)*(cj2)*(sj2)))+(((0.0158604557715362)*(cj3)*(sj3)*(x143)))+(((-0.0158604557715362)*(cj3)*(sj3)*(x145)))+(((-0.416199688533230)*(py)*(pz)*(sj0)))+(((-0.0182978008722355)*(cj3)*(x145)))+(((-0.0733878371602598)*(cj0)*(py)*(pz)))+(((0.00626731379894077)*(cj3)*(sj2)))+(((0.0251936982024400)*(sj3)*(x145)))+(((0.0149232158649733)*(x145)))+(((0.00576687008113338)*(cj2)*(cj3)))+(((-0.00733146142408444)*(sj2)))+(((0.00626731379894077)*(cj2)*(sj3)))))), ((gconst1)*(((((-0.0106047716626493)*(cj2)))+(((0.0115022150354114)*(x144)*(x145)))+(((-0.422620344255005)*(x147)))+(((-0.00435824073612482)*(x145)*(x146)))+(((0.892537757914323)*(cj0)*(px)*(pz)))+(((0.00800127442690355)*(cj3)*(x143)))+(((0.0115022150354114)*(x143)*(x146)))+(((0.00584732064446125)*(cj3)*(sj3)*(x143)))+(((-0.0134402171850868)*(cj3)*(sj2)))+(((0.0100658010547960)*(x143)))+(((0.0157222754412639)*(sj2)))+(((0.0292325715383419)*(cj2)*(cj3)*(sj2)))+(((-0.00739980748067322)*(sj3)*(x145)))+(((-0.00584732064446125)*(cj2)*(sj2)*(x146)))+(((-0.0194360068238237)*(cj2)*(sj2)*(sj3)))+(((0.0317209115430725)*(cj2)*(cj3)*(sj2)*(sj3)))+(((-0.00236354528074851)*(x145)))+(((-0.157379780528043)*(px)*(pz)*(sj0)))+(((-0.00584732064446125)*(cj3)*(sj3)*(x145)))+(((0.0299081249260652)*(pz)))+(((0.0218327640576687)*(sj3)*(x143)))+(((0.892537757914323)*(py)*(pz)*(sj0)))+(((-0.00435824073612482)*(x143)*(x144)))+(((0.157379780528043)*(cj0)*(py)*(pz)))+(((-0.0133289900094151)*(cj2)*(sj2)))+(((0.0123670186071920)*(sj2)*(sj3)))+(((-0.0123670186071920)*(cj2)*(cj3)))+(((-0.0114347323969202)*(cj3)*(x145)))+(((-0.0134402171850868)*(cj2)*(sj3)))+(((0.00584732064446125)*(cj2)*(sj2)*(x144)))))));
sj1array[0]=IKsin(j1array[0]);
cj1array[0]=IKcos(j1array[0]);
if( j1array[0] > IKPI )
{
    j1array[0]-=IK2PI;
}
else if( j1array[0] < -IKPI )
{    j1array[0]+=IK2PI;
}
j1valid[0] = true;
for(int ij1 = 0; ij1 < 1; ++ij1)
{
if( !j1valid[ij1] )
{
    continue;
}
_ij1[0] = ij1; _ij1[1] = -1;
for(int iij1 = ij1+1; iij1 < 1; ++iij1)
{
if( j1valid[iij1] && IKabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && IKabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
{
    j1valid[iij1]=false; _ij1[1] = iij1; break; 
}
}
j1 = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
{
IKReal evalcond[5];
IKReal x148=IKsin(j1);
IKReal x149=IKcos(j1);
evalcond[0]=((((0.0495054651867961)*(sj2)*(sj3)*(x148)))+(((0.133338008689083)*(sj2)*(x149)))+(((-0.140477556470912)*(x148)))+(((0.133338008689083)*(cj2)*(x148)))+(((0.120221499394405)*(cj3)*(sj2)*(x148)))+(((0.120221499394405)*(cj2)*(sj3)*(x148)))+(((-0.0655061533595257)*(x149)))+(((0.0211181305713861)*(sj2)*(x148)))+(((0.120221499394405)*(sj2)*(sj3)*(x149)))+(((-0.0495054651867961)*(cj2)*(sj3)*(x149)))+(((-0.0495054651867961)*(cj2)*(cj3)*(x148)))+(((-0.0211181305713861)*(cj2)*(x149)))+(((-0.0495054651867961)*(cj3)*(sj2)*(x149)))+(((-1.00000000000000)*(pz)))+(((-0.120221499394405)*(cj2)*(cj3)*(x149))));
evalcond[1]=((0.0100150000000000)+(((0.0487877319636934)*(cj0)*(py)*(x149)))+(((0.0114608710120556)*(px)*(sj0)))+(((0.00927151872708020)*(x149)))+(((0.276686704953440)*(py)*(sj0)*(x149)))+(((-0.129021903445301)*(py)*(sj0)*(x148)))+(((-0.0227502295196805)*(cj0)*(py)*(x148)))+(((-0.0487877319636934)*(px)*(sj0)*(x149)))+(((-0.129021903445301)*(cj0)*(px)*(x148)))+(((-0.00812421365107049)*(cj3)))+(((-0.131012306719051)*(pz)*(x149)))+(((-0.280955112941824)*(pz)*(x148)))+(((-0.0649972956025481)*(py)*(sj0)))+(((0.276686704953440)*(cj0)*(px)*(x149)))+(((-0.00432340612172870)*(x148)))+(((-0.0114608710120556)*(cj0)*(py)))+(((0.0341511164173554)*(sj3)))+(((-1.00000000000000)*(pp)))+(((-0.0649972956025481)*(cj0)*(px)))+(((0.0227502295196805)*(px)*(sj0)*(x148))));
evalcond[2]=((((0.0299081249260652)*(x148)))+(((-0.422620344255005)*(pz)*(x148)))+(((0.0956751919860577)*(cj3)*(sj2)))+(((0.892537757914323)*(py)*(sj0)*(x148)))+(((0.0956751919860577)*(cj2)*(sj3)))+(((-0.0880355475840924)*(sj2)*(sj3)))+(((0.157379780528043)*(cj0)*(py)*(x148)))+(((-0.0733878371602598)*(px)*(sj0)*(x149)))+(((0.416199688533230)*(cj0)*(px)*(x149)))+(((-0.157379780528043)*(px)*(sj0)*(x148)))+(((0.0880355475840924)*(cj2)*(cj3)))+(((0.0139464713604152)*(x149)))+(((0.416199688533230)*(py)*(sj0)*(x149)))+(((0.906306815941368)*(pz)*(x149)))+(((0.0733878371602598)*(cj0)*(py)*(x149)))+(((-0.111920194486864)*(sj2)))+(((0.0754908608112439)*(cj2)))+(((0.892537757914323)*(cj0)*(px)*(x148))));
evalcond[3]=((-0.155000000000000)+(((0.0299081249260652)*(x149)))+(((-0.422620344255005)*(pz)*(x149)))+(((0.0956751919860577)*(sj2)*(sj3)))+(((0.0880355475840924)*(cj3)*(sj2)))+(((-0.0733878371602598)*(cj0)*(py)*(x148)))+(((-0.416199688533230)*(py)*(sj0)*(x148)))+(((0.0733878371602598)*(px)*(sj0)*(x148)))+(((0.892537757914323)*(py)*(sj0)*(x149)))+(((-0.0956751919860577)*(cj2)*(cj3)))+(((0.157379780528043)*(cj0)*(py)*(x149)))+(((-0.0139464713604152)*(x148)))+(((0.0754908608112439)*(sj2)))+(((-0.157379780528043)*(px)*(sj0)*(x149)))+(((0.111920194486864)*(cj2)))+(((0.0880355475840924)*(cj2)*(sj3)))+(((-0.416199688533230)*(cj0)*(px)*(x148)))+(((-0.906306815941368)*(pz)*(x148)))+(((0.892537757914323)*(cj0)*(px)*(x149))));
evalcond[4]=((0.0330000000000000)+(((-0.133338008689083)*(sj2)*(x148)))+(((0.0495054651867961)*(sj2)*(sj3)*(x149)))+(((-0.140477556470912)*(x149)))+(((0.0495054651867961)*(cj2)*(sj3)*(x148)))+(((0.0655061533595257)*(x148)))+(((0.133338008689083)*(cj2)*(x149)))+(((-0.120221499394405)*(sj2)*(sj3)*(x148)))+(((0.120221499394405)*(cj3)*(sj2)*(x149)))+(((0.120221499394405)*(cj2)*(sj3)*(x149)))+(((0.0211181305713861)*(sj2)*(x149)))+(((0.984807509129517)*(py)*(sj0)))+(((0.0211181305713861)*(cj2)*(x148)))+(((-0.173649560788721)*(px)*(sj0)))+(((0.120221499394405)*(cj2)*(cj3)*(x148)))+(((-0.0495054651867961)*(cj2)*(cj3)*(x149)))+(((0.984807509129517)*(cj0)*(px)))+(((0.173649560788721)*(cj0)*(py)))+(((0.0495054651867961)*(cj3)*(sj2)*(x148))));
if( IKabs(evalcond[0]) > 0.000001  || IKabs(evalcond[1]) > 0.000001  || IKabs(evalcond[2]) > 0.000001  || IKabs(evalcond[3]) > 0.000001  || IKabs(evalcond[4]) > 0.000001  )
{
continue;
}
}

{
vsolutions.push_back(IKSolution()); IKSolution& solution = vsolutions.back();
solution.basesol.resize(5);
solution.basesol[0].foffset = j0;
solution.basesol[0].indices[0] = _ij0[0];
solution.basesol[0].indices[1] = _ij0[1];
solution.basesol[0].maxsolutions = _nj0;
solution.basesol[1].foffset = j1;
solution.basesol[1].indices[0] = _ij1[0];
solution.basesol[1].indices[1] = _ij1[1];
solution.basesol[1].maxsolutions = _nj1;
solution.basesol[2].foffset = j2;
solution.basesol[2].indices[0] = _ij2[0];
solution.basesol[2].indices[1] = _ij2[1];
solution.basesol[2].maxsolutions = _nj2;
solution.basesol[3].foffset = j3;
solution.basesol[3].indices[0] = _ij3[0];
solution.basesol[3].indices[1] = _ij3[1];
solution.basesol[3].maxsolutions = _nj3;
solution.basesol[4].foffset = j4;
solution.basesol[4].indices[0] = _ij4[0];
solution.basesol[4].indices[1] = _ij4[1];
solution.basesol[4].maxsolutions = _nj4;
solution.vfree.resize(0);
}
}
}

}

}
}
}

}

}
}
}
}
return vsolutions.size()>0;
}
};


/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions) {
IKSolver solver;
return solver.ik(eetrans,eerot,pfree,vsolutions);
}

IKFAST_API const char* getKinematicsHash() { return "<robot:GenericRobot - youbot_with_kinect (e7e417d39e3cf245d6866d62dab808b3)>"; }

IKFAST_API const char* getIKFastVersion() { return "55"; }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>
#ifdef IKFAST_NAMESPACE
using namespace IKFAST_NAMESPACE;
#endif
int main(int argc, char** argv)
{
    if( argc != 12+getNumFreeParameters()+1 ) {
        printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
               "Returns the ik solutions given the transformation of the end effector specified by\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
               "There are %d free parameters that have to be specified.\n\n",getNumFreeParameters());
        return 1;
    }

    std::vector<IKSolution> vsolutions;
    std::vector<IKReal> vfree(getNumFreeParameters());
    IKReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return -1;
    }

    printf("Found %d ik solutions:\n", (int)vsolutions.size());
    std::vector<IKReal> sol(getNumJoints());
    for(std::size_t i = 0; i < vsolutions.size(); ++i) {
        printf("sol%d (free=%d): ", (int)i, (int)vsolutions[i].GetFree().size());
        std::vector<IKReal> vsolfree(vsolutions[i].GetFree().size());
        vsolutions[i].GetSolution(&sol[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < sol.size(); ++j)
            printf("%.15f, ", sol[j]);
        printf("\n");
    }
    return 0;
}

#endif
