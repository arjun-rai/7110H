#include "vex.h"
#include "paths.h"
#include "pure-pursuit.h"
std::vector<std::vector<pathPoint>> convert_to_path(std::string str)
{
    std::istringstream stream(str);
    std::string line;
    std::vector<std::vector<pathPoint>> paths;

    while (std::getline(stream, line)) {
        if (line.find("#PATH-POINTS-START Path") != std::string::npos) {
            // Start a new path for each "#PATH-POINTS-START Path"
            paths.push_back(std::vector<pathPoint>());
            continue;
        }

        std::istringstream lineStream(line);
        std::string token;

        // Skip the first two values and only consider the first two for x, y
        std::getline(lineStream, token, ','); // x
        double x = atof(token.c_str());

        std::getline(lineStream, token, ','); // y
        double y = atof(token.c_str());

        paths.back().push_back(point(x,y));
    }
    return paths;
}
const char * path_str = R"(#PATH-POINTS-START Path
100.201,-135.604,120,0
102.2,-135.666,120
104.2,-135.671,120
106.199,-135.615,120
108.195,-135.488,120
110.185,-135.292,120
112.166,-135.022,120
114.136,-134.674,120
116.089,-134.244,120
118.021,-133.729,120
119.928,-133.125,120
121.803,-132.431,120
123.642,-131.646,120
125.439,-130.768,120
127.188,-129.8,120
128.885,-128.742,120
130.524,-127.595,120
132.098,-126.363,120
133.608,-125.051,120
135.05,-123.665,120
136.419,-122.208,120
137.717,-120.687,120
138.944,-119.107,120
140.1,-117.476,120
141.184,-115.795,120
142.199,-114.072,120
143.149,-112.312,120
144.035,-110.519,120
144.858,-108.697,120
145.621,-106.848,120
146.328,-104.977,120
146.981,-103.087,120
147.583,-101.18,120
148.135,-99.257,120
148.64,-97.322,120
149.099,-95.376,120
149.517,-93.42,120
149.894,-91.456,120
150.233,-89.485,120
150.535,-87.508,120
150.802,-85.526,120
151.036,-83.54,120
151.238,-81.55,120
151.408,-79.557,120
151.549,-77.562,120
151.662,-75.565,120
151.748,-73.567,120
151.807,-71.568,120
151.832,-70.334,120,0
151.832,-70.334,0,0
#PATH-POINTS-START Path
155.391,-107.186,120,0
153.445,-106.727,120
151.491,-106.302,120
149.529,-105.91,120
147.562,-105.55,120
145.589,-105.224,120
143.611,-104.925,120
141.63,-104.653,120
139.645,-104.406,120
137.658,-104.184,120
135.668,-103.984,120
133.676,-103.804,120
131.683,-103.642,120
129.688,-103.499,120
127.692,-103.371,120
125.695,-103.256,120
123.698,-103.154,120
121.7,-103.062,120
119.702,-102.98,120
117.703,-102.904,120
115.704,-102.835,120
113.705,-102.769,120
111.706,-102.706,120
109.707,-102.643,120
107.708,-102.579,120
105.709,-102.511,120
103.711,-102.439,120
101.712,-102.36,120
99.714,-102.272,120
97.717,-102.173,120
95.72,-102.059,120
93.724,-101.93,120
91.73,-101.783,120
89.737,-101.613,120
87.746,-101.42,120
85.758,-101.2,120
83.774,-100.948,120
81.794,-100.665,120
79.821,-100.339,120
77.855,-99.975,120
75.898,-99.561,120
73.952,-99.1,120
72.021,-98.578,120
70.107,-98,120
68.215,-97.352,120
66.348,-96.636,120
64.511,-95.845,120
62.712,-94.971,120
60.955,-94.017,120
59.249,-92.974,120
57.6,-91.842,120
56.013,-90.626,120
54.5,-89.318,120
53.063,-87.928,120
51.705,-86.46,120
50.439,-84.912,120
49.253,-83.302,120
48.163,-81.626,120
47.153,-79.9,120
46.237,-78.122,120
45.397,-76.307,120
44.642,-74.456,120
43.962,-72.575,120
43.35,-70.671,120
42.809,-68.746,120
42.333,-66.803,120
41.915,-64.848,120
41.552,-62.881,120
41.24,-60.906,120
40.977,-58.923,120
40.759,-56.935,120
40.583,-54.943,120
40.448,-52.948,120
40.35,-50.95,120
40.272,-48.446,120,0
40.272,-48.446,0,0
#PATH-POINTS-START Path
40.272,-49.156,120,0
40.268,-47.156,120
40.264,-45.156,120
40.26,-43.156,120
40.257,-41.156,120
40.253,-39.156,120
40.249,-37.156,120
40.245,-35.156,120
40.241,-33.156,120
40.237,-31.156,120
40.233,-29.156,120
40.229,-27.156,120
40.225,-25.156,120
40.221,-23.156,120
40.218,-21.156,120
40.214,-19.156,120
40.21,-17.156,120
40.206,-15.156,120
40.202,-13.156,120
40.198,-11.156,120
40.194,-9.156,120
40.19,-7.156,120
40.186,-5.156,120
40.182,-3.156,120
40.179,-1.156,120
40.175,0.844,120
40.171,2.844,120
40.167,4.844,120
40.163,6.844,120
40.159,8.844,120
40.155,10.844,120
40.151,12.844,120
40.147,14.844,120
40.143,16.844,120
40.14,18.844,120
40.136,20.844,120
40.132,22.844,120
40.128,24.844,120
40.124,26.844,120
40.12,28.844,120
40.116,30.844,120
40.112,32.844,120
40.108,34.844,120
40.104,36.844,120
40.101,38.844,120
40.097,40.844,120
40.093,42.844,120
40.089,44.844,120
40.085,46.844,120
40.081,48.844,120
40.077,50.844,120
40.073,52.844,120
40.069,54.844,120
40.065,56.844,120
40.062,58.844,120
40.058,60.844,120
40.054,62.844,120
40.05,64.844,120
40.046,66.844,120
40.042,68.844,120
40.038,70.844,120
40.034,72.844,120
40.03,74.844,120
40.026,76.844,120
40.023,78.844,120
40.019,80.844,120
40.015,82.844,120
40.011,84.844,120
40.005,87.899,120,0
40.005,87.899,0,0
#PATH-POINTS-START Path
40.005,85.769,120,0
42.002,85.878,120
43.996,86.025,120
45.988,86.213,120
47.974,86.442,120
49.956,86.713,120
51.931,87.027,120
53.898,87.385,120
55.857,87.788,120
57.805,88.244,120
59.74,88.748,120
61.662,89.3,120
63.57,89.901,120
65.457,90.562,120
67.326,91.274,120
69.174,92.039,120
70.995,92.866,120
72.791,93.746,120
74.554,94.688,120
76.287,95.688,120
77.981,96.75,120
79.637,97.871,120
81.247,99.058,120
82.811,100.304,120
84.322,101.613,120
85.776,102.987,120
87.169,104.421,120
88.497,105.917,120
89.751,107.474,120
90.926,109.093,120
92.017,110.769,120
93.016,112.501,120
93.917,114.286,120
94.711,116.121,120
95.389,118.002,120
95.941,119.924,120
96.356,121.88,120
96.626,123.861,120
96.741,125.857,120
96.68,127.856,120
96.444,129.841,120
96.011,131.792,120
95.386,133.691,120
94.551,135.507,120
93.519,137.219,120
92.294,138.798,120
90.891,140.222,120
89.336,141.478,120
87.648,142.549,120
85.859,143.44,120
83.992,144.155,120
82.067,144.694,120
80.104,145.076,120
78.119,145.313,120
76.122,145.418,120
74.122,145.403,120,0
72.123,145.464,120
70.124,145.529,120
68.126,145.596,120
66.127,145.669,120
64.129,145.751,120
62.131,145.857,120
60.146,146.084,120
58.168,146.378,120
56.178,146.583,120
54.186,146.754,120
52.192,146.91,120
50.197,147.054,120
47.683,147.226,120,0
47.683,147.226,0,0
#PATH-POINTS-START Path
47.683,148.646,120,0
45.688,148.511,120
43.692,148.39,120
41.695,148.283,120
39.697,148.19,120
37.698,148.108,120
35.7,148.038,120
33.7,147.981,120
31.701,147.933,120
29.701,147.894,120
27.702,147.863,120
25.702,147.84,120
23.702,147.826,120
21.702,147.819,120
19.702,147.817,120
17.702,147.82,120
15.702,147.83,120
13.702,147.844,120
11.702,147.862,120
9.702,147.883,120
7.702,147.908,120
5.702,147.936,120
3.703,147.966,120
1.703,147.998,120
-0.297,148.033,120
-2.296,148.068,120
-4.296,148.103,120
-6.296,148.14,120
-8.295,148.176,120
-10.295,148.212,120
-12.295,148.247,120
-14.295,148.281,120
-16.294,148.314,120
-18.294,148.345,120
-20.294,148.374,120
-22.294,148.4,120
-24.293,148.424,120
-26.293,148.444,120
-28.293,148.461,120
-30.293,148.473,120
-32.293,148.482,120
-34.293,148.487,120
-36.293,148.484,120
-38.293,148.477,120
-40.293,148.465,120
-42.293,148.446,120
-44.293,148.42,120
-46.293,148.387,120
-48.292,148.348,120
-50.292,148.299,120
-52.291,148.242,120
-54.29,148.177,120
-56.288,148.103,120
-58.287,148.017,120
-60.284,147.921,120
-62.281,147.816,120
-64.278,147.701,120
-66.274,147.57,120
-68.269,147.427,120
-70.263,147.273,120
-72.256,147.106,120
-74.247,146.921,120
-76.237,146.722,120
-78.226,146.509,120
-80.213,146.28,120
-82.197,146.031,120
-84.179,145.764,120
-86.159,145.479,120
-88.136,145.176,120
-90.109,144.853,120
-92.079,144.503,120
-94.044,144.132,120
-96.005,143.739,120
-97.961,143.323,120
-99.912,142.882,120
-101.855,142.41,120
-103.791,141.909,120
-105.72,141.38,120
-107.64,140.822,120
-109.551,140.232,120
-111.452,139.61,120
-113.341,138.953,120
-115.216,138.256,120
-117.076,137.522,120
-118.921,136.75,120
-120.748,135.937,120
-122.557,135.084,120
-124.345,134.187,120
-126.109,133.247,120
-127.849,132.261,120
-129.562,131.228,120
-131.245,130.148,120
-132.895,129.018,120
-134.509,127.836,120
-136.085,126.605,120
-137.621,125.324,120
-139.114,123.993,120
-140.561,122.613,120
-141.96,121.184,120
-143.308,119.708,120
-144.604,118.184,120
-145.844,116.616,120
-147.029,115.005,120
-148.156,113.353,120
-149.224,111.663,120
-150.233,109.936,120
-151.182,108.176,120
-152.073,106.385,120
-152.896,104.563,120
-153.66,102.714,120
-154.365,100.843,120
-155.015,98.952,120
-155.609,97.043,120
-156.151,95.118,120
-156.638,93.178,120
-157.065,91.224,120
-157.444,89.261,120
-157.779,87.289,120
-158.071,85.311,120
-158.307,83.324,120
-158.501,81.334,120
-158.659,79.341,120
-158.779,77.344,120
-158.849,75.345,120
-158.89,73.346,120,0
-158.902,71.265,120
-158.902,71.265,0,0
#PATH-POINTS-START Path
-150.962,92.158,120,0
-148.962,92.135,120
-146.963,92.111,120
-144.963,92.088,120
-142.963,92.065,120
-140.963,92.041,120
-138.963,92.018,120
-136.963,91.994,120
-134.963,91.971,120
-132.964,91.948,120
-130.964,91.924,120
-128.964,91.901,120
-126.964,91.877,120
-124.964,91.854,120
-122.964,91.831,120
-120.964,91.807,120
-118.965,91.784,120
-116.965,91.76,120
-114.965,91.737,120
-112.965,91.714,120
-110.965,91.69,120
-108.965,91.667,120
-106.965,91.644,120
-104.965,91.62,120
-102.966,91.597,120
-100.966,91.573,120
-98.966,91.55,120
-96.966,91.527,120
-94.966,91.503,120
-92.966,91.48,120
-90.966,91.456,120
-88.967,91.433,120
-86.967,91.41,120
-84.967,91.386,120
-82.967,91.363,120
-80.967,91.339,120
-78.967,91.316,120
-76.967,91.293,120
-74.968,91.269,120
-72.968,91.246,120
-70.968,91.223,120
-68.968,91.199,120
-66.968,91.176,120
-64.968,91.152,120
-62.968,91.129,120
-60.968,91.106,120
-58.969,91.082,120
-56.969,91.059,120
-54.969,91.035,120
-52.969,91.012,120
-50.969,90.989,120
-48.969,90.965,120
-46.969,90.942,120
-44.97,90.918,120
-42.97,90.895,120
-40.97,90.872,120
-38.97,90.848,120
-36.97,90.825,120
-34.97,90.801,120
-32.97,90.778,120
-30.971,90.755,120
-29.567,90.738,120,0
-29.567,90.738,0,0
#PATH-POINTS-START Path
-29.567,88.609,120,0
-29.728,86.615,120
-29.902,84.623,120
-30.101,82.633,120
-30.335,80.646,120
-30.615,78.666,120
-30.953,76.695,120
-31.355,74.736,120
-31.832,72.794,120
-32.395,70.875,120
-33.049,68.985,120
-33.798,67.131,120
-34.652,65.323,120
-35.606,63.565,120
-36.665,61.869,120
-37.822,60.238,120
-39.076,58.68,120
-40.417,57.197,120
-41.839,55.791,120
-43.336,54.465,120
-44.898,53.216,120
-46.517,52.043,120
-48.186,50.942,120
-49.9,49.91,120
-51.651,48.945,120
-53.437,48.044,120
-55.25,47.2,120
-57.087,46.41,120
-58.947,45.674,120
-60.823,44.983,120
-62.717,44.341,120
-64.624,43.737,120
-66.543,43.175,120
-68.473,42.65,120
-70.411,42.158,120
-72.358,41.699,120
-74.312,41.272,120
-76.272,40.873,120
-78.237,40.5,120
-80.206,40.153,120
-82.18,39.83,120
-84.157,39.529,120
-86.138,39.251,120
-88.121,38.994,120
-90.107,38.756,120
-92.095,38.536,120
-94.085,38.335,120
-96.076,38.15,120
-98.069,37.982,120
-100.063,37.829,120
-102.058,37.691,120
-104.054,37.566,120
-106.051,37.455,120
-108.049,37.356,120
-110.047,37.268,120
-112.046,37.195,120
-114.045,37.134,120
-116.044,37.082,120
-118.044,37.04,120
-120.043,37.008,120
-122.043,36.989,120
-125.536,36.973,120,0
-125.536,36.973,0,0
#PATH-POINTS-START Path
-124.116,38.393,120,0
-122.116,38.39,120
-120.116,38.381,120
-118.116,38.367,120
-116.116,38.349,120
-114.116,38.327,120
-112.117,38.301,120
-110.117,38.273,120
-108.117,38.244,120
-106.117,38.214,120
-104.117,38.185,120
-102.118,38.158,120
-100.118,38.136,120
-98.118,38.12,120
-96.118,38.111,120
-94.118,38.113,120
-92.118,38.128,120
-90.118,38.159,120
-88.119,38.21,120
-86.12,38.283,120
-84.123,38.383,120
-82.127,38.513,120
-80.134,38.678,120
-78.144,38.883,120
-76.16,39.132,120
-74.182,39.427,120
-72.213,39.776,120
-70.254,40.179,120
-68.308,40.64,120
-66.377,41.162,120
-64.464,41.744,120
-62.57,42.388,120
-60.699,43.092,120
-58.85,43.855,120
-57.026,44.675,120
-55.227,45.549,120
-53.454,46.474,120
-51.706,47.446,120
-49.984,48.462,120
-48.286,49.519,120
-46.611,50.613,120
-44.96,51.742,120
-43.331,52.901,120
-41.721,54.088,120
-40.131,55.301,120
-38.56,56.539,120
-37.006,57.797,120
-35.467,59.075,120
-33.944,60.371,120
-32.434,61.683,120
-30.938,63.01,120
-28.59,65.14,120,0
-28.59,65.14,0,0
#PATH-POINTS-START Path
-27.88,63.721,120,0
-27.888,61.721,120
-27.897,59.721,120
-27.905,57.721,120
-27.914,55.721,120
-27.922,53.721,120
-27.931,51.721,120
-27.939,49.721,120
-27.947,47.721,120
-27.956,45.721,120
-27.964,43.721,120
-27.973,41.721,120
-27.981,39.721,120
-27.99,37.721,120
-27.998,35.721,120
-28.007,33.721,120
-28.015,31.721,120
-28.024,29.721,120
-28.032,27.721,120
-28.041,25.721,120
-28.049,23.721,120
-28.058,21.721,120
-28.066,19.721,120
-28.074,17.721,120
-28.083,15.721,120
-28.091,13.721,120
-28.1,11.721,120
-28.108,9.721,120
-28.117,7.721,120
-28.125,5.721,120
-28.134,3.721,120
-28.142,1.721,120
-28.151,-0.279,120
-28.159,-2.279,120
-28.168,-4.279,120
-28.176,-6.279,120
-28.185,-8.279,120
-28.193,-10.279,120
-28.201,-12.279,120
-28.21,-14.279,120
-28.218,-16.279,120
-28.227,-18.279,120
-28.235,-20.279,120
-28.244,-22.279,120
-28.252,-24.279,120
-28.261,-26.279,120
-28.269,-28.279,120
-28.278,-30.252,120,0
-28.278,-30.252,0,0
#PATH-POINTS-START Path
-29.697,-30.252,120,0
-31.697,-30.245,120
-33.697,-30.224,120
-35.697,-30.194,120
-37.697,-30.156,120
-39.696,-30.111,120
-41.696,-30.061,120
-43.695,-30.005,120
-45.694,-29.946,120
-47.693,-29.883,120
-49.692,-29.816,120
-51.691,-29.747,120
-53.689,-29.675,120
-55.688,-29.601,120
-57.686,-29.525,120
-59.685,-29.447,120
-61.683,-29.367,120
-63.682,-29.286,120
-65.68,-29.204,120
-67.678,-29.12,120
-69.676,-29.035,120
-71.675,-28.949,120
-73.673,-28.862,120
-75.671,-28.775,120
-77.669,-28.687,120
-79.667,-28.598,120
-81.665,-28.508,120
-83.663,-28.419,120
-85.661,-28.329,120
-87.659,-28.238,120
-89.657,-28.148,120
-91.655,-28.057,120
-93.653,-27.967,120
-95.651,-27.876,120
-97.649,-27.786,120
-99.647,-27.696,120
-101.645,-27.607,120
-103.643,-27.519,120
-105.641,-27.431,120
-107.639,-27.344,120
-109.637,-27.258,120
-111.635,-27.174,120
-113.633,-27.091,120
-115.632,-27.01,120
-117.63,-26.932,120
-119.629,-26.857,120
-121.628,-26.785,120
-124.116,-26.703,120,0
-124.116,-26.703,0,0
#PATH-POINTS-START Path
-123.008,-15.79,120,0
-121.008,-15.812,120
-119.009,-15.835,120
-117.009,-15.857,120
-115.009,-15.88,120
-113.009,-15.902,120
-111.009,-15.925,120
-109.009,-15.947,120
-107.009,-15.97,120
-105.009,-15.992,120
-103.01,-16.015,120
-101.01,-16.037,120
-99.01,-16.06,120
-97.01,-16.082,120
-95.01,-16.105,120
-93.01,-16.127,120
-91.01,-16.149,120
-89.01,-16.172,120
-87.011,-16.194,120
-85.011,-16.217,120
-83.011,-16.239,120
-81.011,-16.262,120
-79.011,-16.284,120
-77.011,-16.307,120
-75.011,-16.329,120
-73.012,-16.352,120
-71.012,-16.374,120
-69.012,-16.397,120
-67.012,-16.419,120
-65.012,-16.442,120
-63.012,-16.464,120
-61.012,-16.487,120
-59.826,-16.5,120,0
-59.826,-16.5,0,0
#PATH-POINTS-START Path
-59.826,-16.5,120,0
-60.195,-18.466,120
-60.57,-20.43,120
-60.959,-22.392,120
-61.359,-24.351,120
-61.767,-26.309,120
-62.193,-28.263,120
-62.625,-30.216,120
-63.071,-32.166,120
-63.529,-34.113,120
-63.994,-36.058,120
-64.477,-37.999,120
-64.968,-39.937,120
-65.47,-41.873,120
-65.987,-43.805,120
-66.511,-45.736,120
-67.05,-47.662,120
-67.6,-49.584,120
-68.158,-51.505,120
-68.735,-53.42,120
-69.319,-55.333,120
-69.914,-57.242,120
-70.526,-59.146,120
-71.146,-61.048,120
-71.779,-62.945,120
-72.426,-64.837,120
-73.083,-66.726,120
-73.753,-68.611,120
-74.439,-70.489,120
-75.134,-72.365,120
-75.842,-74.235,120
-76.567,-76.099,120
-77.303,-77.959,120
-78.049,-79.814,120
-78.816,-81.662,120
-79.594,-83.504,120
-80.384,-85.341,120
-81.19,-87.172,120
-82.013,-88.994,120
-82.85,-90.811,120
-83.7,-92.621,120
-84.568,-94.423,120
-85.454,-96.216,120
-86.356,-98.001,120
-87.273,-99.778,120
-88.208,-101.547,120
-89.164,-103.303,120
-90.14,-105.049,120
-91.134,-106.784,120
-92.148,-108.508,120
-93.183,-110.219,120
-94.241,-111.917,120
-95.321,-113.6,120
-96.428,-115.266,120
-97.561,-116.914,120
-98.721,-118.543,120
-99.91,-120.151,120
-101.129,-121.737,120
-102.38,-123.297,120
-103.667,-124.828,120
-104.992,-126.326,120
-106.36,-127.785,120
-107.772,-129.201,120
-109.23,-130.569,120
-110.739,-131.882,120
-112.309,-133.12,120
-113.941,-134.275,120
-115.639,-135.331,120
-117.412,-136.255,120
-119.258,-137.021,120
-121.174,-137.591,120
-123.146,-137.913,120
-125.144,-137.93,120
-127.117,-137.614,120
-129.003,-136.959,120
-130.753,-135.995,120
-132.338,-134.78,120
-133.759,-133.375,120
-135.019,-131.823,120
-136.14,-130.168,120
-137.133,-128.433,120
-138.025,-126.643,120
-138.826,-124.81,120
-139.539,-122.942,120
-140.181,-121.048,120
-140.761,-119.134,120
-141.286,-117.205,120
-141.762,-115.262,120
-142.191,-113.309,120
-142.576,-111.346,120
-142.924,-109.377,120
-143.24,-107.402,120
-143.527,-105.423,120
-143.786,-103.439,120
-144.01,-101.452,120
-144.214,-99.463,120
-144.399,-97.471,120
-144.554,-95.477,120
-144.691,-93.482,120
-144.815,-91.486,120
-144.911,-89.488,120
-144.995,-87.49,120
-145.063,-85.491,120
-145.112,-83.492,120
-145.155,-81.492,120
-145.172,-79.492,120
-145.185,-77.492,120
-145.18,-75.492,120
-145.166,-73.492,120
-145.138,-71.493,120
-145.1,-69.493,120
-145.051,-67.494,120
-144.992,-65.494,120,0
-144.882,-62.327,120
-144.882,-62.327,0,0)";

std::vector<std::vector<pathPoint>> pathMain = convert_to_path(path_str);
std::vector<double> finSpeed = {400, 500, 0, 600, 600, 0, 500, 0, 0, 400, 0, 500};
std::vector<double> startSpeed = {500, 600, 500, 200, 600, 500, 200, 400, 500, 200, 500, 300};
//