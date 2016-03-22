#include "trigtables.h"
#include <stdint.h>
#include <math.h>

const float COSTABLE[TABLEN]=     // lookup table for cos and sin functions
{
-9.999803e-001,-9.999210e-001,-9.998224e-001,-9.996842e-001,-9.995066e-001,-9.992895e-001,-9.990329e-001,-9.987370e-001,-9.984016e-001,-9.980267e-001,-9.976125e-001,-9.971589e-001,-9.966659e-001,-9.961336e-001,-9.955620e-001,-9.949510e-001,-9.943008e-001,-9.936113e-001,-9.928826e-001,-9.921147e-001,-9.913076e-001,-9.904614e-001,-9.895761e-001,-9.886518e-001,-9.876883e-001,-9.866860e-001,-9.856446e-001,-9.845643e-001,-9.834452e-001,-9.822873e-001,-9.810905e-001,-9.798551e-001,-9.785809e-001,-9.772681e-001,-9.759168e-001,-9.745269e-001,-9.730985e-001,-9.716317e-001,-9.701266e-001,-9.685832e-001,-9.670015e-001,-9.653816e-001,-9.637237e-001,-9.620277e-001,-9.602937e-001,-9.585218e-001,-9.567121e-001,-9.548646e-001,-9.529794e-001,-9.510565e-001,-9.490962e-001,-9.470983e-001,-9.450631e-001,-9.429905e-001,-9.408808e-001,-9.387339e-001,-9.365499e-001,-9.343290e-001,-9.320711e-001,-9.297765e-001,-9.274452e-001,-9.250772e-001,-9.226728e-001,-9.202319e-001,-9.177546e-001,-9.152412e-001,-9.126916e-001,-9.101060e-001,-9.074844e-001,-9.048271e-001,-9.021340e-001,-8.994053e-001,-8.966411e-001,-8.938414e-001,-8.910065e-001,-8.881365e-001,-8.852313e-001,-8.822912e-001,-8.793163e-001,-8.763067e-001,-8.732625e-001,-8.701838e-001,-8.670707e-001,-8.639234e-001,-8.607420e-001,-8.575267e-001,-8.542775e-001,-8.509945e-001,-8.476780e-001,-8.443279e-001,-8.409446e-001,-8.375281e-001,-8.340785e-001,-8.305959e-001,-8.270806e-001,-8.235326e-001,-8.199521e-001,-8.163393e-001,-8.126942e-001,-8.090170e-001,-8.053079e-001,-8.015670e-001,-7.977945e-001,-7.939904e-001,-7.901550e-001,-7.862885e-001,-7.823908e-001,-7.784623e-001,-7.745031e-001,-7.705133e-001,-7.664930e-001,-7.624425e-001,-7.583619e-001,-7.542514e-001,-7.501111e-001,-7.459412e-001,-7.417418e-001,-7.375131e-001,-7.332554e-001,-7.289687e-001,-7.246532e-001,-7.203090e-001,-7.159365e-001,-7.115357e-001,-7.071068e-001,-7.026500e-001,-6.981654e-001,-6.936533e-001,-6.891138e-001,-6.845471e-001,-6.799534e-001,-6.753328e-001,-6.706856e-001,-6.660119e-001,-6.613119e-001,-6.565858e-001,-6.518338e-001,-6.470560e-001,-6.422527e-001,-6.374240e-001,-6.325702e-001,-6.276914e-001,-6.227878e-001,-6.178596e-001,-6.129071e-001,-6.079303e-001,-6.029296e-001,-5.979050e-001,-5.928568e-001,-5.877853e-001,-5.826905e-001,-5.775727e-001,-5.724322e-001,-5.672690e-001,-5.620834e-001,-5.568756e-001,-5.516459e-001,-5.463944e-001,-5.411213e-001,-5.358268e-001,-5.305112e-001,-5.251747e-001,-5.198174e-001,-5.144396e-001,-5.090414e-001,-5.036232e-001,-4.981851e-001,-4.927274e-001,-4.872502e-001,-4.817537e-001,-4.762382e-001,-4.707040e-001,-4.651511e-001,-4.595799e-001,-4.539905e-001,-4.483832e-001,-4.427583e-001,-4.371158e-001,-4.314561e-001,-4.257793e-001,-4.200858e-001,-4.143756e-001,-4.086491e-001,-4.029065e-001,-3.971479e-001,-3.913737e-001,-3.855840e-001,-3.797791e-001,-3.739592e-001,-3.681246e-001,-3.622754e-001,-3.564119e-001,-3.505343e-001,-3.446429e-001,-3.387379e-001,-3.328196e-001,-3.268881e-001,-3.209436e-001,-3.149865e-001,-3.090170e-001,-3.030353e-001,-2.970416e-001,-2.910362e-001,-2.850193e-001,-2.789911e-001,-2.729520e-001,-2.669020e-001,-2.608415e-001,-2.547708e-001,-2.486899e-001,-2.425993e-001,-2.364990e-001,-2.303895e-001,-2.242708e-001,-2.181433e-001,-2.120071e-001,-2.058626e-001,-1.997100e-001,-1.935495e-001,-1.873813e-001,-1.812058e-001,-1.750231e-001,-1.688335e-001,-1.626372e-001,-1.564345e-001,-1.502256e-001,-1.440108e-001,-1.377903e-001,-1.315644e-001,-1.253333e-001,-1.190972e-001,-1.128564e-001,-1.066112e-001,-1.003617e-001,-9.410834e-002,-8.785122e-002,-8.159064e-002,-7.532683e-002,-6.906005e-002,-6.279054e-002,-5.651856e-002,-5.024434e-002,-4.396814e-002,-3.769021e-002,-3.141078e-002,-2.513012e-002,-1.884846e-002,-1.256606e-002,-6.283167e-003,-2.320510e-008,6.283121e-003,1.256602e-002,1.884842e-002,2.513007e-002,3.141074e-002,3.769016e-002,4.396810e-002,5.024430e-002,5.651851e-002,6.279050e-002,6.906000e-002,7.532678e-002,8.159059e-002,8.785117e-002,9.410829e-002,1.003617e-001,1.066111e-001,1.128564e-001,1.190971e-001,1.253332e-001,1.315643e-001,1.377903e-001,1.440108e-001,1.502256e-001,1.564344e-001,1.626371e-001,1.688334e-001,1.750230e-001,1.812057e-001,1.873813e-001,1.935494e-001,1.997100e-001,2.058626e-001,2.120071e-001,2.181432e-001,2.242707e-001,2.303894e-001,2.364990e-001,2.425992e-001,2.486899e-001,2.547707e-001,2.608415e-001,2.669020e-001,2.729519e-001,2.789911e-001,2.850192e-001,2.910361e-001,2.970416e-001,3.030353e-001,3.090170e-001,3.149865e-001,3.209436e-001,3.268880e-001,3.328195e-001,3.387379e-001,3.446429e-001,3.505343e-001,3.564119e-001,3.622754e-001,3.681245e-001,3.739592e-001,3.797791e-001,3.855840e-001,3.913737e-001,3.971479e-001,4.029064e-001,4.086491e-001,4.143756e-001,4.200857e-001,4.257793e-001,4.314560e-001,4.371158e-001,4.427582e-001,4.483832e-001,4.539905e-001,4.595798e-001,4.651511e-001,4.707039e-001,4.762382e-001,4.817537e-001,4.872501e-001,4.927273e-001,4.981851e-001,5.036232e-001,5.090414e-001,5.144395e-001,5.198173e-001,5.251746e-001,5.305112e-001,5.358268e-001,5.411212e-001,5.463943e-001,5.516459e-001,5.568756e-001,5.620834e-001,5.672689e-001,5.724321e-001,5.775727e-001,5.826905e-001,5.877852e-001,5.928568e-001,5.979050e-001,6.029295e-001,6.079303e-001,6.129070e-001,6.178596e-001,6.227878e-001,6.276914e-001,6.325702e-001,6.374240e-001,6.422526e-001,6.470560e-001,6.518337e-001,6.565857e-001,6.613119e-001,6.660119e-001,6.706856e-001,6.753328e-001,6.799534e-001,6.845471e-001,6.891138e-001,6.936533e-001,6.981654e-001,7.026500e-001,7.071068e-001,7.115357e-001,7.159365e-001,7.203090e-001,7.246531e-001,7.289686e-001,7.332553e-001,7.375131e-001,7.417418e-001,7.459411e-001,7.501111e-001,7.542514e-001,7.583619e-001,7.624425e-001,7.664930e-001,7.705132e-001,7.745031e-001,7.784623e-001,7.823908e-001,7.862884e-001,7.901550e-001,7.939904e-001,7.977944e-001,8.015670e-001,8.053079e-001,8.090170e-001,8.126942e-001,8.163392e-001,8.199521e-001,8.235326e-001,8.270806e-001,8.305959e-001,8.340784e-001,8.375280e-001,8.409446e-001,8.443279e-001,8.476779e-001,8.509945e-001,8.542774e-001,8.575267e-001,8.607420e-001,8.639234e-001,8.670707e-001,8.701838e-001,8.732625e-001,8.763067e-001,8.793163e-001,8.822912e-001,8.852313e-001,8.881364e-001,8.910065e-001,8.938414e-001,8.966410e-001,8.994052e-001,9.021340e-001,9.048270e-001,9.074844e-001,9.101060e-001,9.126916e-001,9.152412e-001,9.177546e-001,9.202318e-001,9.226727e-001,9.250772e-001,9.274452e-001,9.297765e-001,9.320711e-001,9.343289e-001,9.365499e-001,9.387339e-001,9.408808e-001,9.429905e-001,9.450631e-001,9.470983e-001,9.490961e-001,9.510565e-001,9.529793e-001,9.548645e-001,9.567121e-001,9.585218e-001,9.602937e-001,9.620277e-001,9.637237e-001,9.653816e-001,9.670015e-001,9.685832e-001,9.701266e-001,9.716317e-001,9.730985e-001,9.745269e-001,9.759168e-001,9.772681e-001,9.785809e-001,9.798551e-001,9.810905e-001,9.822873e-001,9.834452e-001,9.845643e-001,9.856446e-001,9.866859e-001,9.876883e-001,9.886517e-001,9.895761e-001,9.904614e-001,9.913076e-001,9.921147e-001,9.928826e-001,9.936113e-001,9.943008e-001,9.949510e-001,9.955620e-001,9.961336e-001,9.966659e-001,9.971589e-001,9.976125e-001,9.980267e-001,9.984016e-001,9.987370e-001,9.990329e-001,9.992895e-001,9.995066e-001,9.996842e-001,9.998224e-001,9.999210e-001,9.999803e-001,1.000000e+000,9.999803e-001,9.999210e-001,9.998224e-001,9.996842e-001,9.995066e-001,9.992895e-001,9.990329e-001,9.987370e-001,9.984016e-001,9.980267e-001,9.976125e-001,9.971589e-001,9.966659e-001,9.961336e-001,9.955620e-001,9.949510e-001,9.943008e-001,9.936113e-001,9.928826e-001,9.921147e-001,9.913076e-001,9.904614e-001,9.895761e-001,9.886517e-001,9.876883e-001,9.866859e-001,9.856446e-001,9.845643e-001,9.834452e-001,9.822873e-001,9.810905e-001,9.798551e-001,9.785809e-001,9.772681e-001,9.759168e-001,9.745269e-001,9.730985e-001,9.716317e-001,9.701266e-001,9.685832e-001,9.670015e-001,9.653816e-001,9.637237e-001,9.620277e-001,9.602937e-001,9.585218e-001,9.567121e-001,9.548645e-001,9.529793e-001,9.510565e-001,9.490961e-001,9.470983e-001,9.450631e-001,9.429905e-001,9.408808e-001,9.387339e-001,9.365499e-001,9.343289e-001,9.320711e-001,9.297765e-001,9.274452e-001,9.250772e-001,9.226727e-001,9.202318e-001,9.177546e-001,9.152412e-001,9.126916e-001,9.101060e-001,9.074844e-001,9.048270e-001,9.021340e-001,8.994052e-001,8.966410e-001,8.938414e-001,8.910065e-001,8.881364e-001,8.852313e-001,8.822912e-001,8.793163e-001,8.763067e-001,8.732625e-001,8.701838e-001,8.670707e-001,8.639234e-001,8.607420e-001,8.575267e-001,8.542774e-001,8.509945e-001,8.476779e-001,8.443279e-001,8.409446e-001,8.375280e-001,8.340784e-001,8.305959e-001,8.270806e-001,8.235326e-001,8.199521e-001,8.163392e-001,8.126942e-001,8.090170e-001,8.053079e-001,8.015670e-001,7.977944e-001,7.939904e-001,7.901550e-001,7.862884e-001,7.823908e-001,7.784623e-001,7.745031e-001,7.705132e-001,7.664930e-001,7.624425e-001,7.583619e-001,7.542514e-001,7.501111e-001,7.459411e-001,7.417418e-001,7.375131e-001,7.332553e-001,7.289686e-001,7.246531e-001,7.203090e-001,7.159365e-001,7.115357e-001,7.071068e-001,7.026500e-001,6.981654e-001,6.936533e-001,6.891138e-001,6.845471e-001,6.799534e-001,6.753328e-001,6.706856e-001,6.660119e-001,6.613119e-001,6.565857e-001,6.518337e-001,6.470560e-001,6.422526e-001,6.374240e-001,6.325702e-001,6.276914e-001,6.227878e-001,6.178596e-001,6.129070e-001,6.079303e-001,6.029295e-001,5.979050e-001,5.928568e-001,5.877852e-001,5.826905e-001,5.775727e-001,5.724321e-001,5.672689e-001,5.620834e-001,5.568756e-001,5.516459e-001,5.463943e-001,5.411212e-001,5.358268e-001,5.305112e-001,5.251746e-001,5.198173e-001,5.144395e-001,5.090414e-001,5.036232e-001,4.981851e-001,4.927273e-001,4.872501e-001,4.817537e-001,4.762382e-001,4.707039e-001,4.651511e-001,4.595798e-001,4.539905e-001,4.483832e-001,4.427582e-001,4.371158e-001,4.314560e-001,4.257793e-001,4.200857e-001,4.143756e-001,4.086491e-001,4.029064e-001,3.971479e-001,3.913737e-001,3.855840e-001,3.797791e-001,3.739592e-001,3.681245e-001,3.622754e-001,3.564119e-001,3.505343e-001,3.446429e-001,3.387379e-001,3.328195e-001,3.268880e-001,3.209436e-001,3.149865e-001,3.090170e-001,3.030353e-001,2.970416e-001,2.910361e-001,2.850192e-001,2.789911e-001,2.729519e-001,2.669020e-001,2.608415e-001,2.547707e-001,2.486899e-001,2.425992e-001,2.364990e-001,2.303894e-001,2.242707e-001,2.181432e-001,2.120071e-001,2.058626e-001,1.997100e-001,1.935494e-001,1.873813e-001,1.812057e-001,1.750230e-001,1.688334e-001,1.626371e-001,1.564344e-001,1.502256e-001,1.440108e-001,1.377903e-001,1.315643e-001,1.253332e-001,1.190971e-001,1.128564e-001,1.066111e-001,1.003617e-001,9.410829e-002,8.785117e-002,8.159059e-002,7.532678e-002,6.906000e-002,6.279050e-002,5.651851e-002,5.024430e-002,4.396810e-002,3.769016e-002,3.141074e-002,2.513007e-002,1.884842e-002,1.256602e-002,6.283121e-003,-2.320510e-008,-6.283167e-003,-1.256606e-002,-1.884846e-002,-2.513012e-002,-3.141078e-002,-3.769021e-002,-4.396814e-002,-5.024434e-002,-5.651856e-002,-6.279054e-002,-6.906005e-002,-7.532683e-002,-8.159064e-002,-8.785122e-002,-9.410834e-002,-1.003617e-001,-1.066112e-001,-1.128564e-001,-1.190972e-001,-1.253333e-001,-1.315644e-001,-1.377903e-001,-1.440108e-001,-1.502256e-001,-1.564345e-001,-1.626372e-001,-1.688335e-001,-1.750231e-001,-1.812058e-001,-1.873813e-001,-1.935495e-001,-1.997100e-001,-2.058626e-001,-2.120071e-001,-2.181433e-001,-2.242708e-001,-2.303895e-001,-2.364990e-001,-2.425993e-001,-2.486899e-001,-2.547708e-001,-2.608415e-001,-2.669020e-001,-2.729520e-001,-2.789911e-001,-2.850193e-001,-2.910362e-001,-2.970416e-001,-3.030353e-001,-3.090170e-001,-3.149865e-001,-3.209436e-001,-3.268881e-001,-3.328196e-001,-3.387379e-001,-3.446429e-001,-3.505343e-001,-3.564119e-001,-3.622754e-001,-3.681246e-001,-3.739592e-001,-3.797791e-001,-3.855840e-001,-3.913737e-001,-3.971479e-001,-4.029065e-001,-4.086491e-001,-4.143756e-001,-4.200858e-001,-4.257793e-001,-4.314561e-001,-4.371158e-001,-4.427583e-001,-4.483832e-001,-4.539905e-001,-4.595799e-001,-4.651511e-001,-4.707040e-001,-4.762382e-001,-4.817537e-001,-4.872502e-001,-4.927274e-001,-4.981851e-001,-5.036232e-001,-5.090414e-001,-5.144396e-001,-5.198174e-001,-5.251747e-001,-5.305112e-001,-5.358268e-001,-5.411213e-001,-5.463944e-001,-5.516459e-001,-5.568756e-001,-5.620834e-001,-5.672690e-001,-5.724322e-001,-5.775727e-001,-5.826905e-001,-5.877853e-001,-5.928568e-001,-5.979050e-001,-6.029296e-001,-6.079303e-001,-6.129071e-001,-6.178596e-001,-6.227878e-001,-6.276914e-001,-6.325702e-001,-6.374240e-001,-6.422527e-001,-6.470560e-001,-6.518338e-001,-6.565858e-001,-6.613119e-001,-6.660119e-001,-6.706856e-001,-6.753328e-001,-6.799534e-001,-6.845471e-001,-6.891138e-001,-6.936533e-001,-6.981654e-001,-7.026500e-001,-7.071068e-001,-7.115357e-001,-7.159365e-001,-7.203090e-001,-7.246532e-001,-7.289687e-001,-7.332554e-001,-7.375131e-001,-7.417418e-001,-7.459412e-001,-7.501111e-001,-7.542514e-001,-7.583619e-001,-7.624425e-001,-7.664930e-001,-7.705133e-001,-7.745031e-001,-7.784623e-001,-7.823908e-001,-7.862885e-001,-7.901550e-001,-7.939904e-001,-7.977945e-001,-8.015670e-001,-8.053079e-001,-8.090170e-001,-8.126942e-001,-8.163393e-001,-8.199521e-001,-8.235326e-001,-8.270806e-001,-8.305959e-001,-8.340785e-001,-8.375281e-001,-8.409446e-001,-8.443279e-001,-8.476780e-001,-8.509945e-001,-8.542775e-001,-8.575267e-001,-8.607420e-001,-8.639234e-001,-8.670707e-001,-8.701838e-001,-8.732625e-001,-8.763067e-001,-8.793163e-001,-8.822912e-001,-8.852313e-001,-8.881365e-001,-8.910065e-001,-8.938414e-001,-8.966411e-001,-8.994053e-001,-9.021340e-001,-9.048271e-001,-9.074844e-001,-9.101060e-001,-9.126916e-001,-9.152412e-001,-9.177546e-001,-9.202319e-001,-9.226728e-001,-9.250772e-001,-9.274452e-001,-9.297765e-001,-9.320711e-001,-9.343290e-001,-9.365499e-001,-9.387339e-001,-9.408808e-001,-9.429905e-001,-9.450631e-001,-9.470983e-001,-9.490962e-001,-9.510565e-001,-9.529794e-001,-9.548646e-001,-9.567121e-001,-9.585218e-001,-9.602937e-001,-9.620277e-001,-9.637237e-001,-9.653816e-001,-9.670015e-001,-9.685832e-001,-9.701266e-001,-9.716317e-001,-9.730985e-001,-9.745269e-001,-9.759168e-001,-9.772681e-001,-9.785809e-001,-9.798551e-001,-9.810905e-001,-9.822873e-001,-9.834452e-001,-9.845643e-001,-9.856446e-001,-9.866860e-001,-9.876883e-001,-9.886518e-001,-9.895761e-001,-9.904614e-001,-9.913076e-001,-9.921147e-001,-9.928826e-001,-9.936113e-001,-9.943008e-001,-9.949510e-001,-9.955620e-001,-9.961336e-001,-9.966659e-001,-9.971589e-001,-9.976125e-001,-9.980267e-001,-9.984016e-001,-9.987370e-001,-9.990329e-001,-9.992895e-001,-9.995066e-001,-9.996842e-001,-9.998224e-001,-9.999210e-001,-9.999803e-001,-1.000000e+000
};

const float ATANTABLE[TABLEN]=     // lookup table for atan function
{
-1.526015e+000,-1.525926e+000,-1.525835e+000,-1.525745e+000,-1.525654e+000,-1.525563e+000,-1.525471e+000,-1.525379e+000,-1.525287e+000,-1.525194e+000,-1.525101e+000,-1.525007e+000,-1.524913e+000,-1.524819e+000,-1.524724e+000,-1.524629e+000,-1.524534e+000,-1.524438e+000,-1.524342e+000,-1.524245e+000,-1.524148e+000,-1.524051e+000,-1.523953e+000,-1.523855e+000,-1.523756e+000,-1.523657e+000,-1.523557e+000,-1.523457e+000,-1.523357e+000,-1.523256e+000,-1.523155e+000,-1.523053e+000,-1.522951e+000,-1.522849e+000,-1.522746e+000,-1.522642e+000,-1.522539e+000,-1.522434e+000,-1.522330e+000,-1.522224e+000,-1.522119e+000,-1.522013e+000,-1.521906e+000,-1.521799e+000,-1.521691e+000,-1.521583e+000,-1.521475e+000,-1.521366e+000,-1.521257e+000,-1.521147e+000,-1.521036e+000,-1.520925e+000,-1.520814e+000,-1.520702e+000,-1.520590e+000,-1.520477e+000,-1.520364e+000,-1.520250e+000,-1.520135e+000,-1.520020e+000,-1.519905e+000,-1.519789e+000,-1.519672e+000,-1.519555e+000,-1.519438e+000,-1.519320e+000,-1.519201e+000,-1.519082e+000,-1.518962e+000,-1.518842e+000,-1.518721e+000,-1.518599e+000,-1.518477e+000,-1.518355e+000,-1.518231e+000,-1.518108e+000,-1.517983e+000,-1.517858e+000,-1.517733e+000,-1.517607e+000,-1.517480e+000,-1.517353e+000,-1.517225e+000,-1.517096e+000,-1.516967e+000,-1.516837e+000,-1.516707e+000,-1.516576e+000,-1.516444e+000,-1.516312e+000,-1.516179e+000,-1.516046e+000,-1.515911e+000,-1.515776e+000,-1.515641e+000,-1.515505e+000,-1.515368e+000,-1.515230e+000,-1.515092e+000,-1.514953e+000,-1.514813e+000,-1.514673e+000,-1.514532e+000,-1.514390e+000,-1.514247e+000,-1.514104e+000,-1.513960e+000,-1.513816e+000,-1.513670e+000,-1.513524e+000,-1.513377e+000,-1.513229e+000,-1.513081e+000,-1.512932e+000,-1.512782e+000,-1.512631e+000,-1.512480e+000,-1.512327e+000,-1.512174e+000,-1.512020e+000,-1.511865e+000,-1.511710e+000,-1.511554e+000,-1.511396e+000,-1.511238e+000,-1.511080e+000,-1.510920e+000,-1.510759e+000,-1.510598e+000,-1.510435e+000,-1.510272e+000,-1.510108e+000,-1.509943e+000,-1.509777e+000,-1.509611e+000,-1.509443e+000,-1.509274e+000,-1.509105e+000,-1.508934e+000,-1.508763e+000,-1.508591e+000,-1.508417e+000,-1.508243e+000,-1.508068e+000,-1.507892e+000,-1.507714e+000,-1.507536e+000,-1.507357e+000,-1.507177e+000,-1.506995e+000,-1.506813e+000,-1.506630e+000,-1.506445e+000,-1.506260e+000,-1.506073e+000,-1.505886e+000,-1.505697e+000,-1.505507e+000,-1.505316e+000,-1.505124e+000,-1.504931e+000,-1.504737e+000,-1.504541e+000,-1.504345e+000,-1.504147e+000,-1.503948e+000,-1.503748e+000,-1.503546e+000,-1.503344e+000,-1.503140e+000,-1.502935e+000,-1.502729e+000,-1.502521e+000,-1.502313e+000,-1.502102e+000,-1.501891e+000,-1.501678e+000,-1.501465e+000,-1.501249e+000,-1.501033e+000,-1.500815e+000,-1.500595e+000,-1.500375e+000,-1.500152e+000,-1.499929e+000,-1.499704e+000,-1.499478e+000,-1.499250e+000,-1.499021e+000,-1.498790e+000,-1.498558e+000,-1.498324e+000,-1.498089e+000,-1.497852e+000,-1.497614e+000,-1.497374e+000,-1.497132e+000,-1.496889e+000,-1.496645e+000,-1.496398e+000,-1.496150e+000,-1.495901e+000,-1.495650e+000,-1.495397e+000,-1.495142e+000,-1.494886e+000,-1.494628e+000,-1.494368e+000,-1.494106e+000,-1.493843e+000,-1.493578e+000,-1.493311e+000,-1.493042e+000,-1.492771e+000,-1.492498e+000,-1.492224e+000,-1.491947e+000,-1.491669e+000,-1.491388e+000,-1.491106e+000,-1.490821e+000,-1.490535e+000,-1.490247e+000,-1.489956e+000,-1.489663e+000,-1.489368e+000,-1.489072e+000,-1.488772e+000,-1.488471e+000,-1.488168e+000,-1.487862e+000,-1.487554e+000,-1.487243e+000,-1.486931e+000,-1.486616e+000,-1.486298e+000,-1.485979e+000,-1.485657e+000,-1.485332e+000,-1.485005e+000,-1.484675e+000,-1.484343e+000,-1.484008e+000,-1.483671e+000,-1.483331e+000,-1.482989e+000,-1.482643e+000,-1.482295e+000,-1.481945e+000,-1.481591e+000,-1.481235e+000,-1.480875e+000,-1.480513e+000,-1.480148e+000,-1.479780e+000,-1.479409e+000,-1.479035e+000,-1.478658e+000,-1.478278e+000,-1.477895e+000,-1.477509e+000,-1.477119e+000,-1.476726e+000,-1.476330e+000,-1.475930e+000,-1.475527e+000,-1.475121e+000,-1.474711e+000,-1.474298e+000,-1.473881e+000,-1.473460e+000,-1.473036e+000,-1.472608e+000,-1.472176e+000,-1.471741e+000,-1.471301e+000,-1.470858e+000,-1.470411e+000,-1.469960e+000,-1.469505e+000,-1.469045e+000,-1.468582e+000,-1.468114e+000,-1.467642e+000,-1.467166e+000,-1.466685e+000,-1.466200e+000,-1.465710e+000,-1.465216e+000,-1.464717e+000,-1.464213e+000,-1.463704e+000,-1.463191e+000,-1.462673e+000,-1.462149e+000,-1.461621e+000,-1.461087e+000,-1.460549e+000,-1.460005e+000,-1.459455e+000,-1.458900e+000,-1.458340e+000,-1.457774e+000,-1.457202e+000,-1.456625e+000,-1.456041e+000,-1.455452e+000,-1.454857e+000,-1.454255e+000,-1.453647e+000,-1.453033e+000,-1.452413e+000,-1.451786e+000,-1.451152e+000,-1.450511e+000,-1.449864e+000,-1.449210e+000,-1.448548e+000,-1.447879e+000,-1.447203e+000,-1.446520e+000,-1.445829e+000,-1.445131e+000,-1.444424e+000,-1.443710e+000,-1.442987e+000,-1.442256e+000,-1.441517e+000,-1.440770e+000,-1.440014e+000,-1.439249e+000,-1.438475e+000,-1.437692e+000,-1.436899e+000,-1.436098e+000,-1.435286e+000,-1.434465e+000,-1.433634e+000,-1.432793e+000,-1.431941e+000,-1.431079e+000,-1.430207e+000,-1.429323e+000,-1.428428e+000,-1.427522e+000,-1.426605e+000,-1.425676e+000,-1.424734e+000,-1.423781e+000,-1.422815e+000,-1.421836e+000,-1.420845e+000,-1.419840e+000,-1.418822e+000,-1.417790e+000,-1.416744e+000,-1.415684e+000,-1.414609e+000,-1.413520e+000,-1.412415e+000,-1.411295e+000,-1.410159e+000,-1.409006e+000,-1.407838e+000,-1.406652e+000,-1.405449e+000,-1.404229e+000,-1.402990e+000,-1.401733e+000,-1.400458e+000,-1.399163e+000,-1.397849e+000,-1.396514e+000,-1.395159e+000,-1.393783e+000,-1.392386e+000,-1.390966e+000,-1.389524e+000,-1.388059e+000,-1.386570e+000,-1.385057e+000,-1.383520e+000,-1.381957e+000,-1.380368e+000,-1.378752e+000,-1.377109e+000,-1.375438e+000,-1.373738e+000,-1.372009e+000,-1.370250e+000,-1.368460e+000,-1.366638e+000,-1.364783e+000,-1.362895e+000,-1.360972e+000,-1.359014e+000,-1.357020e+000,-1.354988e+000,-1.352918e+000,-1.350808e+000,-1.348658e+000,-1.346466e+000,-1.344232e+000,-1.341953e+000,-1.339628e+000,-1.337257e+000,-1.334838e+000,-1.332369e+000,-1.329848e+000,-1.327275e+000,-1.324648e+000,-1.321964e+000,-1.319222e+000,-1.316421e+000,-1.313558e+000,-1.310631e+000,-1.307638e+000,-1.304578e+000,-1.301447e+000,-1.298243e+000,-1.294965e+000,-1.291608e+000,-1.288171e+000,-1.284651e+000,-1.281045e+000,-1.277349e+000,-1.273560e+000,-1.269675e+000,-1.265691e+000,-1.261603e+000,-1.257408e+000,-1.253101e+000,-1.248678e+000,-1.244136e+000,-1.239467e+000,-1.234669e+000,-1.229735e+000,-1.224660e+000,-1.219438e+000,-1.214063e+000,-1.208528e+000,-1.202827e+000,-1.196952e+000,-1.190895e+000,-1.184649e+000,-1.178206e+000,-1.171555e+000,-1.164687e+000,-1.157593e+000,-1.150262e+000,-1.142682e+000,-1.134842e+000,-1.126729e+000,-1.118329e+000,-1.109629e+000,-1.100612e+000,-1.091265e+000,-1.081568e+000,-1.071505e+000,-1.061057e+000,-1.050202e+000,-1.038921e+000,-1.027190e+000,-1.014985e+000,-1.002281e+000,-9.890501e-001,-9.752651e-001,-9.608955e-001,-9.459096e-001,-9.302740e-001,-9.139538e-001,-8.969123e-001,-8.791110e-001,-8.605103e-001,-8.410687e-001,-8.207436e-001,-7.994914e-001,-7.772677e-001,-7.540280e-001,-7.297277e-001,-7.043233e-001,-6.777729e-001,-6.500372e-001,-6.210807e-001,-5.908728e-001,-5.593892e-001,-5.266141e-001,-4.925409e-001,-4.571748e-001,-4.205343e-001,-3.826530e-001,-3.435808e-001,-3.033854e-001,-2.621529e-001,-2.199880e-001,-1.770131e-001,-1.333677e-001,-8.920534e-002,-4.469158e-002,0.000000e+000,4.469158e-002,8.920534e-002,1.333677e-001,1.770131e-001,2.199880e-001,2.621529e-001,3.033854e-001,3.435808e-001,3.826530e-001,4.205343e-001,4.571748e-001,4.925409e-001,5.266141e-001,5.593892e-001,5.908728e-001,6.210807e-001,6.500372e-001,6.777729e-001,7.043233e-001,7.297277e-001,7.540280e-001,7.772677e-001,7.994914e-001,8.207436e-001,8.410687e-001,8.605103e-001,8.791110e-001,8.969123e-001,9.139538e-001,9.302740e-001,9.459096e-001,9.608955e-001,9.752651e-001,9.890501e-001,1.002281e+000,1.014985e+000,1.027190e+000,1.038921e+000,1.050202e+000,1.061057e+000,1.071505e+000,1.081568e+000,1.091265e+000,1.100612e+000,1.109629e+000,1.118329e+000,1.126729e+000,1.134842e+000,1.142682e+000,1.150262e+000,1.157593e+000,1.164687e+000,1.171555e+000,1.178206e+000,1.184649e+000,1.190895e+000,1.196952e+000,1.202827e+000,1.208528e+000,1.214063e+000,1.219438e+000,1.224660e+000,1.229735e+000,1.234669e+000,1.239467e+000,1.244136e+000,1.248678e+000,1.253101e+000,1.257408e+000,1.261603e+000,1.265691e+000,1.269675e+000,1.273560e+000,1.277349e+000,1.281045e+000,1.284651e+000,1.288171e+000,1.291608e+000,1.294965e+000,1.298243e+000,1.301447e+000,1.304578e+000,1.307638e+000,1.310631e+000,1.313558e+000,1.316421e+000,1.319222e+000,1.321964e+000,1.324648e+000,1.327275e+000,1.329848e+000,1.332369e+000,1.334838e+000,1.337257e+000,1.339628e+000,1.341953e+000,1.344232e+000,1.346466e+000,1.348658e+000,1.350808e+000,1.352918e+000,1.354988e+000,1.357020e+000,1.359014e+000,1.360972e+000,1.362895e+000,1.364783e+000,1.366638e+000,1.368460e+000,1.370250e+000,1.372009e+000,1.373738e+000,1.375438e+000,1.377109e+000,1.378752e+000,1.380368e+000,1.381957e+000,1.383520e+000,1.385057e+000,1.386570e+000,1.388059e+000,1.389524e+000,1.390966e+000,1.392386e+000,1.393783e+000,1.395159e+000,1.396514e+000,1.397849e+000,1.399163e+000,1.400458e+000,1.401733e+000,1.402990e+000,1.404229e+000,1.405449e+000,1.406652e+000,1.407838e+000,1.409006e+000,1.410159e+000,1.411295e+000,1.412415e+000,1.413520e+000,1.414609e+000,1.415684e+000,1.416744e+000,1.417790e+000,1.418822e+000,1.419840e+000,1.420845e+000,1.421836e+000,1.422815e+000,1.423781e+000,1.424734e+000,1.425676e+000,1.426605e+000,1.427522e+000,1.428428e+000,1.429323e+000,1.430207e+000,1.431079e+000,1.431941e+000,1.432793e+000,1.433634e+000,1.434465e+000,1.435286e+000,1.436098e+000,1.436899e+000,1.437692e+000,1.438475e+000,1.439249e+000,1.440014e+000,1.440770e+000,1.441517e+000,1.442256e+000,1.442987e+000,1.443710e+000,1.444424e+000,1.445131e+000,1.445829e+000,1.446520e+000,1.447203e+000,1.447879e+000,1.448548e+000,1.449210e+000,1.449864e+000,1.450511e+000,1.451152e+000,1.451786e+000,1.452413e+000,1.453033e+000,1.453647e+000,1.454255e+000,1.454857e+000,1.455452e+000,1.456041e+000,1.456625e+000,1.457202e+000,1.457774e+000,1.458340e+000,1.458900e+000,1.459455e+000,1.460005e+000,1.460549e+000,1.461087e+000,1.461621e+000,1.462149e+000,1.462673e+000,1.463191e+000,1.463704e+000,1.464213e+000,1.464717e+000,1.465216e+000,1.465710e+000,1.466200e+000,1.466685e+000,1.467166e+000,1.467642e+000,1.468114e+000,1.468582e+000,1.469045e+000,1.469505e+000,1.469960e+000,1.470411e+000,1.470858e+000,1.471301e+000,1.471741e+000,1.472176e+000,1.472608e+000,1.473036e+000,1.473460e+000,1.473881e+000,1.474298e+000,1.474711e+000,1.475121e+000,1.475527e+000,1.475930e+000,1.476330e+000,1.476726e+000,1.477119e+000,1.477509e+000,1.477895e+000,1.478278e+000,1.478658e+000,1.479035e+000,1.479409e+000,1.479780e+000,1.480148e+000,1.480513e+000,1.480875e+000,1.481235e+000,1.481591e+000,1.481945e+000,1.482295e+000,1.482643e+000,1.482989e+000,1.483331e+000,1.483671e+000,1.484008e+000,1.484343e+000,1.484675e+000,1.485005e+000,1.485332e+000,1.485657e+000,1.485979e+000,1.486298e+000,1.486616e+000,1.486931e+000,1.487243e+000,1.487554e+000,1.487862e+000,1.488168e+000,1.488471e+000,1.488772e+000,1.489072e+000,1.489368e+000,1.489663e+000,1.489956e+000,1.490247e+000,1.490535e+000,1.490821e+000,1.491106e+000,1.491388e+000,1.491669e+000,1.491947e+000,1.492224e+000,1.492498e+000,1.492771e+000,1.493042e+000,1.493311e+000,1.493578e+000,1.493843e+000,1.494106e+000,1.494368e+000,1.494628e+000,1.494886e+000,1.495142e+000,1.495397e+000,1.495650e+000,1.495901e+000,1.496150e+000,1.496398e+000,1.496645e+000,1.496889e+000,1.497132e+000,1.497374e+000,1.497614e+000,1.497852e+000,1.498089e+000,1.498324e+000,1.498558e+000,1.498790e+000,1.499021e+000,1.499250e+000,1.499478e+000,1.499704e+000,1.499929e+000,1.500152e+000,1.500375e+000,1.500595e+000,1.500815e+000,1.501033e+000,1.501249e+000,1.501465e+000,1.501678e+000,1.501891e+000,1.502102e+000,1.502313e+000,1.502521e+000,1.502729e+000,1.502935e+000,1.503140e+000,1.503344e+000,1.503546e+000,1.503748e+000,1.503948e+000,1.504147e+000,1.504345e+000,1.504541e+000,1.504737e+000,1.504931e+000,1.505124e+000,1.505316e+000,1.505507e+000,1.505697e+000,1.505886e+000,1.506073e+000,1.506260e+000,1.506445e+000,1.506630e+000,1.506813e+000,1.506995e+000,1.507177e+000,1.507357e+000,1.507536e+000,1.507714e+000,1.507892e+000,1.508068e+000,1.508243e+000,1.508417e+000,1.508591e+000,1.508763e+000,1.508934e+000,1.509105e+000,1.509274e+000,1.509443e+000,1.509611e+000,1.509777e+000,1.509943e+000,1.510108e+000,1.510272e+000,1.510435e+000,1.510598e+000,1.510759e+000,1.510920e+000,1.511080e+000,1.511238e+000,1.511396e+000,1.511554e+000,1.511710e+000,1.511865e+000,1.512020e+000,1.512174e+000,1.512327e+000,1.512480e+000,1.512631e+000,1.512782e+000,1.512932e+000,1.513081e+000,1.513229e+000,1.513377e+000,1.513524e+000,1.513670e+000,1.513816e+000,1.513960e+000,1.514104e+000,1.514247e+000,1.514390e+000,1.514532e+000,1.514673e+000,1.514813e+000,1.514953e+000,1.515092e+000,1.515230e+000,1.515368e+000,1.515505e+000,1.515641e+000,1.515776e+000,1.515911e+000,1.516046e+000,1.516179e+000,1.516312e+000,1.516444e+000,1.516576e+000,1.516707e+000,1.516837e+000,1.516967e+000,1.517096e+000,1.517225e+000,1.517353e+000,1.517480e+000,1.517607e+000,1.517733e+000,1.517858e+000,1.517983e+000,1.518108e+000,1.518231e+000,1.518355e+000,1.518477e+000,1.518599e+000,1.518721e+000,1.518842e+000,1.518962e+000,1.519082e+000,1.519201e+000,1.519320e+000,1.519438e+000,1.519555e+000,1.519672e+000,1.519789e+000,1.519905e+000,1.520020e+000,1.520135e+000,1.520250e+000,1.520364e+000,1.520477e+000,1.520590e+000,1.520702e+000,1.520814e+000,1.520925e+000,1.521036e+000,1.521147e+000,1.521257e+000,1.521366e+000,1.521475e+000,1.521583e+000,1.521691e+000,1.521799e+000,1.521906e+000,1.522013e+000,1.522119e+000,1.522224e+000,1.522330e+000,1.522434e+000,1.522539e+000,1.522642e+000,1.522746e+000,1.522849e+000,1.522951e+000,1.523053e+000,1.523155e+000,1.523256e+000,1.523357e+000,1.523457e+000,1.523557e+000,1.523657e+000,1.523756e+000,1.523855e+000,1.523953e+000,1.524051e+000,1.524148e+000,1.524245e+000,1.524342e+000,1.524438e+000,1.524534e+000,1.524629e+000,1.524724e+000,1.524819e+000,1.524913e+000,1.525007e+000,1.525101e+000,1.525194e+000,1.525287e+000,1.525379e+000,1.525471e+000,1.525563e+000,1.525654e+000,1.525745e+000,1.525835e+000,1.525926e+000,1.526015e+000,1.526105e+000
};

//************************************************************************
//************************************************************************
float atan2tab(float y,float x)  // note output is 0 to 2*pi
{
 int16_t Index;
 float th;

  if (fabsf(y) > fabsf(x*ATANRANGE))
  {
    if (y < 0) th = -1.570796f;
    else th =  1.570796f;
  }
  else
  {
    Index = (int16_t)((y/x/ATANRANGE + 1)*TABLEN/2.0f);
    if (Index < 0) Index=0;
    if (Index > TABLEN-1) Index=TABLEN-1;

    th=ATANTABLE[Index];

    if ((y<0)&&(x<0)) th -= 3.141593f;
    else if (x<0) th += 3.141593f;
  }

  if(th<0) th += 6.283185f; // change range to 0 to 2*pi

  return(th);
}

//************************************************************************
//************************************************************************
void sincostab(float theta, float *sth, float *cth) // note theta range -pi to 2*pi
{
  int16_t CosIndex, SinIndex;

  if (theta > 3.141593f) theta -= 6.283185f;

  CosIndex = (int16_t)(theta*TABLEN/2.0f/3.141593f + TABLEN/2.0f);
  if (CosIndex < 0) CosIndex=0;
  if (CosIndex > TABLEN-1) CosIndex=TABLEN-1;

  SinIndex=CosIndex-TABLEN/4;
  if (SinIndex < 0) SinIndex += TABLEN;

  *cth=COSTABLE[CosIndex];
  *sth=COSTABLE[SinIndex];
}
