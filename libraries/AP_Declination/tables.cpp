// this is an auto-generated file from the IGRF tables. Do not edit
// To re-generate run generate/generate.py

#include "AP_Declination.h"

const float AP_Declination::SAMPLING_RES = 10;
const float AP_Declination::SAMPLING_MIN_LAT = -90;
const float AP_Declination::SAMPLING_MAX_LAT = 90;
const float AP_Declination::SAMPLING_MIN_LON = -180;
const float AP_Declination::SAMPLING_MAX_LON = 180;

const float AP_Declination::declination_table[19][37] = {
    {149.39017,139.39017,129.39017,119.39017,109.39017,99.39017,89.39017,79.39017,69.39017,59.39017,49.39017,39.39017,29.39017,19.39017,9.39017,-0.60983,-10.60983,-20.60983,-30.60983,-40.60983,-50.60983,-60.60983,-70.60983,-80.60983,-90.60983,-100.60983,-110.60983,-120.60983,-130.60983,-140.60983,-150.60983,-160.60983,-170.60983,179.39017,169.39017,159.39017,149.39017},
    {129.68055,117.40579,106.24477,96.04773,86.62784,77.80195,69.41061,61.32573,53.45169,45.72214,38.09383,30.53811,23.03147,15.54682,8.04781,0.48783,-7.18586,-15.02441,-23.06903,-31.34539,-39.86304,-48.62024,-57.61278,-66.84455,-76.33743,-86.13915,-96.32825,-107.01528,-118.33795,-130.44483,-143.45867,-157.41186,-172.16402,172.64871,157.56102,143.11341,129.68055},
    {85.55836,77.64281,71.28695,65.85360,60.92878,56.19567,51.39767,46.34289,40.92199,35.12049,29.01545,22.75232,16.50064,10.39321,4.46626,-1.37073,-7.31219,-13.58396,-20.35484,-27.67487,-35.46622,-43.56456,-51.78573,-59.98653,-68.10107,-76.15459,-84.27052,-92.69385,-101.86681,-112.63343,-126.76991,-148.17517,177.65681,138.77313,112.22682,96.22663,85.55836},
    {47.49819,46.23363,44.79414,43.39720,42.08500,40.72761,39.03070,36.61509,33.15929,28.52794,22.83910,16.46982,9.98520,3.96268,-1.25299,-5.73383,-9.95542,-14.57199,-20.10731,-26.71818,-34.14329,-41.86447,-49.35580,-56.24164,-62.31263,-67.46007,-71.57734,-74.42386,-75.37816,-72.73097,-60.78346,-21.38154,25.79189,42.34187,47.18711,48.12874,47.49819},
    {30.87416,31.09390,30.84469,30.45577,30.15626,30.04601,29.94666,29.32571,27.46222,23.76590,18.06615,10.80189,3.02742,-3.94991,-9.22241,-12.73515,-15.20416,-17.71196,-21.33864,-26.69755,-33.45453,-40.56786,-47.03192,-52.21555,-55.74279,-57.29632,-56.42231,-52.30831,-43.81742,-30.41520,-13.92260,1.72301,13.76541,21.91413,26.95159,29.70168,30.87416},
    {22.29318,22.81601,22.90296,22.73877,22.48482,22.36424,22.48150,22.51730,21.61878,18.69336,13.02032,4.88872,-4.15422,-12.00761,-17.37082,-20.33628,-21.73490,-22.29220,-22.99021,-25.49921,-30.41140,-36.29191,-41.40575,-44.71313,-45.62833,-43.80182,-39.00779,-31.18858,-21.26789,-11.41044,-3.03812,3.96779,9.92375,14.81262,18.53045,20.98688,22.29318},
    {16.78613,17.27117,17.49492,17.50942,17.27945,16.91492,16.66520,16.54223,15.85875,13.27180,7.62897,-0.84546,-10.13065,-17.71101,-22.41864,-24.72770,-25.51615,-24.74009,-22.28625,-20.25447,-21.42002,-25.34822,-29.50837,-31.86310,-31.47615,-28.38496,-23.07671,-16.13813,-8.99777,-3.44550,0.45860,3.91867,7.47084,10.84985,13.70890,15.71294,16.78613},
    {13.12632,13.38846,13.54280,13.64342,13.51501,13.07452,12.57865,12.21291,11.38985,8.71902,2.99910,-5.31364,-13.88196,-20.36629,-23.91442,-24.94952,-24.16104,-21.42580,-16.61372,-11.52739,-9.15807,-10.68788,-14.36652,-17.28461,-17.76333,-15.91726,-12.40704,-7.70418,-3.06799,-0.10590,1.46326,3.22015,5.68791,8.33612,10.70076,12.35566,13.12632},
    {10.86841,10.85036,10.79238,10.86595,10.82171,10.43549,9.96291,9.55221,8.49215,5.48656,-0.31461,-8.08194,-15.51401,-20.68113,-22.77800,-21.93754,-18.96672,-14.65317,-9.71053,-5.10304,-2.04312,-1.80362,-4.23836,-7.17507,-8.60754,-8.25081,-6.50892,-3.63546,-0.67774,0.84673,1.23510,2.20541,4.24485,6.61378,8.78767,10.30401,10.86841},
    {9.66103,9.47651,9.22137,9.26425,9.31013,9.02198,8.62108,8.11944,6.66692,3.20032,-2.59292,-9.60357,-15.82136,-19.60880,-20.15263,-17.70497,-13.48778,-8.91720,-4.96864,-1.77224,0.73280,1.65328,0.32738,-2.01999,-3.62506,-4.00597,-3.38437,-1.85367,-0.14925,0.46618,0.24151,0.82415,2.69290,5.05212,7.34214,9.04234,9.66103},
    {8.97422,9.00814,8.80965,8.95913,9.18880,9.03842,8.55217,7.62739,5.49436,1.43027,-4.37384,-10.61486,-15.56839,-17.90809,-17.16401,-13.98476,-9.65308,-5.44191,-2.25945,0.00437,1.92129,2.97040,2.26376,0.44650,-1.00665,-1.60638,-1.59572,-1.05743,-0.42835,-0.58680,-1.28496,-1.07810,0.56189,2.98705,5.62943,7.86947,8.97422},
    {8.05180,8.89751,9.27338,9.80842,10.35020,10.37873,9.67136,8.03405,4.93480,0.04358,-5.98358,-11.58789,-15.26024,-16.22519,-14.62751,-11.34561,-7.35740,-3.54507,-0.70734,1.15932,2.67882,3.65625,3.30952,1.93393,0.70323,0.07561,-0.24759,-0.46554,-0.85662,-1.89368,-3.20711,-3.53334,-2.28017,0.13364,3.14629,6.06806,8.05180},
    {6.49606,8.58795,10.07815,11.33284,12.26601,12.44744,11.50996,9.14933,4.98625,-0.95568,-7.50860,-12.75509,-15.39748,-15.31246,-13.18841,-9.93514,-6.22423,-2.63770,0.16213,2.01864,3.38641,4.32864,4.36459,3.55324,2.63207,1.96296,1.29112,0.32048,-1.17859,-3.30546,-5.43548,-6.38127,-5.54553,-3.20878,0.05008,3.53564,6.49606},
    {4.70266,8.03047,10.80534,12.99194,14.39903,14.69825,13.53913,10.53284,5.30130,-1.86779,-9.23419,-14.48616,-16.56132,-15.85973,-13.37614,-10.00199,-6.28462,-2.66034,0.36872,2.57562,4.19928,5.43547,6.11749,6.13286,5.67313,4.86749,3.56083,1.51512,-1.37780,-4.82935,-7.88391,-9.37574,-8.81024,-6.48248,-3.01997,0.90444,4.70266},
    {3.34886,7.58839,11.39312,14.48867,16.51511,17.06997,15.71777,11.93542,5.30543,-3.52297,-12.03686,-17.56494,-19.40469,-18.36897,-15.57612,-11.88368,-7.82564,-3.79527,-0.16368,2.85661,5.33862,7.44719,9.15517,10.23620,10.42697,9.49517,7.23627,3.55420,-1.30412,-6.45567,-10.50853,-12.35554,-11.77223,-9.23335,-5.45519,-1.09987,3.34886},
    {2.70234,7.57117,12.07616,15.90446,18.61760,19.62862,18.15071,13.18615,4.09488,-7.60304,-17.76882,-23.48853,-24.91203,-23.37164,-20.05319,-15.75121,-10.96973,-6.06527,-1.31975,3.08529,7.10166,10.73932,13.90736,16.30735,17.44524,16.71317,13.52290,7.63691,-0.17743,-7.84778,-13.15963,-15.20157,-14.31846,-11.34511,-7.11680,-2.28785,2.70234},
    {2.32589,7.74101,12.81590,17.22582,20.47716,21.74627,19.61411,11.91286,-2.59796,-19.19983,-30.25194,-34.47731,-34.10896,-31.05721,-26.45879,-20.96544,-14.97173,-8.74224,-2.47208,3.69009,9.62601,15.21302,20.26578,24.46751,27.29390,27.91262,25.09095,17.50301,5.52181,-6.67074,-14.52112,-17.28420,-16.31933,-13.03383,-8.42857,-3.16449,2.32589},
    {1.22917,6.59202,11.52261,15.43360,17.31209,15.12231,4.84661,-16.80837,-38.99533,-49.78596,-51.96223,-49.68155,-45.06886,-39.14386,-32.43382,-25.23801,-17.74333,-10.07902,-2.34477,5.37278,12.98796,20.40337,27.49255,34.07199,39.84984,44.32263,46.55504,44.73225,35.69755,17.62715,-1.30100,-11.60909,-14.32118,-12.77316,-9.00771,-4.12457,1.22917},
    {179.64910,-170.35090,-160.35090,-150.35090,-140.35090,-130.35090,-120.35090,-110.35090,-100.35090,-90.35090,-80.35090,-70.35090,-60.35090,-50.35090,-40.35090,-30.35090,-20.35090,-10.35090,-0.35090,9.64910,19.64910,29.64910,39.64910,49.64910,59.64910,69.64910,79.64910,89.64910,99.64910,109.64910,119.64910,129.64910,139.64910,149.64910,159.64910,169.64910,179.64910}
};

const float AP_Declination::inclination_table[19][37] = {
    {-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14644,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645,-72.14645},
    {-78.40867,-77.64602,-76.72609,-75.69091,-74.57665,-73.41604,-72.24107,-71.08441,-69.97919,-68.95704,-68.04507,-67.26285,-66.62062,-66.11974,-65.75557,-65.52189,-65.41541,-65.43876,-65.60086,-65.91464,-66.39296,-67.04409,-67.86802,-68.85435,-69.98204,-71.22030,-72.53013,-73.86560,-75.17478,-76.40041,-77.48115,-78.35480,-78.96534,-79.27359,-79.26766,-78.96581,-78.40867},
    {-81.00216,-79.18176,-77.35158,-75.49267,-73.57960,-71.59646,-69.55230,-67.49357,-65.50745,-63.71064,-62.22272,-61.13007,-60.45378,-60.13822,-60.07071,-60.12624,-60.21640,-60.32045,-60.48840,-60.81944,-61.42635,-62.39844,-63.77625,-65.54624,-67.65349,-70.02189,-72.57042,-75.21991,-77.89139,-80.49752,-82.92049,-84.94393,-86.09159,-85.82294,-84.50624,-82.80428,-81.00216},
    {-77.56639,-75.56896,-73.64863,-71.74427,-69.77089,-67.63038,-65.24748,-62.62950,-59.91803,-57.39557,-55.42751,-54.33802,-54.25458,-55.01279,-56.21054,-57.39035,-58.22402,-58.60101,-58.62683,-58.57756,-58.81307,-59.64415,-61.21483,-63.47950,-66.27404,-69.41088,-72.73658,-76.13592,-79.50560,-82.72284,-85.56529,-87.23938,-86.33449,-84.19030,-81.89843,-79.67524,-77.56639},
    {-71.61228,-69.67664,-67.80806,-65.98204,-64.14179,-62.16866,-59.88775,-57.16793,-54.07884,-51.00778,-48.64764,-47.76854,-48.77468,-51.35405,-54.63142,-57.69399,-59.95206,-61.13765,-61.22175,-60.50251,-59.67262,-59.55049,-60.61795,-62.83476,-65.82498,-69.15816,-72.50741,-75.62237,-78.21521,-79.94576,-80.58294,-80.18566,-79.04021,-77.43973,-75.58233,-73.60333,-71.61228},
    {-64.37080,-62.41593,-60.47181,-58.52203,-56.58522,-54.64814,-52.56753,-50.08681,-47.05031,-43.75074,-41.15830,-40.63140,-42.95234,-47.52581,-52.88249,-57.82236,-61.82364,-64.62080,-65.80757,-65.17282,-63.31312,-61.59303,-61.26251,-62.63093,-65.14731,-68.00802,-70.60736,-72.54178,-73.50938,-73.53679,-73.00980,-72.21533,-71.17305,-69.83715,-68.20210,-66.33354,-64.37080},
    {-54.95126,-52.86073,-50.76014,-48.57424,-46.33777,-44.17355,-42.09010,-39.77941,-36.80805,-33.28839,-30.50844,-30.53562,-34.44901,-41.10471,-48.36259,-54.85228,-60.28512,-64.63129,-67.35104,-67.74221,-65.88149,-62.96263,-60.80965,-60.55491,-61.85784,-63.66840,-65.19073,-65.94450,-65.64988,-64.62712,-63.59171,-62.78001,-61.89599,-60.68105,-59.05259,-57.06905,-54.95126},
    {-42.11295,-39.71059,-37.41646,-35.04641,-32.53710,-30.07688,-27.79961,-25.32097,-22.02265,-18.06273,-15.20161,-16.09754,-21.84814,-30.77991,-40.21560,-48.37003,-54.77795,-59.57150,-62.53583,-63.18387,-61.44015,-58.07223,-54.77080,-53.13014,-53.22317,-54.08175,-54.90993,-55.09934,-54.18254,-52.60616,-51.42291,-50.82171,-50.12302,-48.88951,-47.04096,-44.66049,-42.11295},
    {-25.13296,-22.25809,-19.80088,-17.41056,-14.83097,-12.26343,-9.85247,-7.05652,-3.31861,0.80536,3.24176,1.42679,-5.53083,-16.11992,-27.56433,-37.27842,-44.09979,-48.19812,-50.07986,-50.02748,-47.99886,-44.31348,-40.50766,-38.32086,-37.92761,-38.42072,-39.09010,-39.24710,-38.21113,-36.49707,-35.50899,-35.37477,-34.94898,-33.62663,-31.39707,-28.37951,-25.13296},
    {-4.98581,-1.66259,0.82055,2.99724,5.34885,7.71033,9.99042,12.78097,16.36301,19.82669,21.31792,18.99613,12.16687,1.66017,-10.13565,-20.13615,-26.58886,-29.58408,-30.18688,-29.38399,-27.14134,-23.29976,-19.27502,-16.97215,-16.54358,-16.99672,-17.69844,-18.04624,-17.28921,-15.91701,-15.48448,-16.05674,-16.13278,-14.93643,-12.49065,-8.92443,-4.98581},
    {14.90426,18.28942,20.62173,22.45839,24.41662,26.46645,28.53131,30.96848,33.77986,36.08537,36.56361,34.12894,28.35788,19.72889,10.02490,1.76877,-3.42425,-5.38537,-5.11486,-3.88849,-1.76758,1.67771,5.32299,7.39801,7.75534,7.37369,6.80267,6.39608,6.70530,7.35765,7.06445,5.78870,4.96803,5.50114,7.50129,10.90027,14.90426},
    {31.19124,34.08505,36.16578,37.78540,39.50269,41.43200,43.46851,45.62160,47.69098,48.99849,48.70415,46.29427,41.76486,35.66177,29.18792,23.77636,20.37895,19.28349,19.91149,21.20554,22.94401,25.46868,28.11803,29.65986,29.93691,29.69506,29.39186,29.16786,29.21284,29.21504,28.36018,26.64796,25.13632,24.69741,25.64821,28.01913,31.19124},
    {43.44020,45.49676,47.27117,48.86193,50.59277,52.58357,54.71495,56.79142,58.48965,59.26055,58.56504,56.28779,52.80821,48.77545,44.92409,41.86876,40.00598,39.53065,40.18377,41.31902,42.64207,44.25840,45.88562,46.90221,47.17943,47.13665,47.09951,47.11775,47.11679,46.76535,45.60069,43.69641,41.77768,40.56922,40.46045,41.53622,43.44020},
    {53.15275,54.40131,55.85749,57.48231,59.33885,61.41926,63.58771,65.61543,67.15522,67.72287,66.94521,64.93360,62.24365,59.50260,57.16044,55.45031,54.48788,54.33019,54.84470,55.69322,56.62878,57.61508,58.57178,59.29857,59.72782,60.00281,60.26066,60.48157,60.48549,59.95954,58.65809,56.72703,54.68939,53.09246,52.27738,52.34540,53.15275},
    {61.95209,62.66563,63.82753,65.37397,67.23268,69.28554,71.36153,73.23315,74.57585,74.98318,74.22100,72.51170,70.39268,68.36634,66.73119,65.59962,64.98913,64.87465,65.16134,65.68035,66.28179,66.91011,67.56286,68.22889,68.89393,69.55035,70.15543,70.58065,70.60455,69.99122,68.66701,66.84394,64.92423,63.29641,62.21294,61.77254,61.95209},
    {70.64756,71.10046,71.99040,73.26707,74.84729,76.60657,78.37678,79.93080,80.95432,81.11206,80.31687,78.86238,77.17454,75.58377,74.27860,73.33582,72.76345,72.52944,72.57157,72.80951,73.17458,73.63910,74.21746,74.93817,75.80695,76.77296,77.70116,78.36293,78.48549,77.89859,76.67690,75.10132,73.49956,72.13288,71.16283,70.66297,70.64756},
    {78.94528,79.24251,79.84077,80.70445,81.77893,82.98410,84.20006,85.23272,85.77970,85.57550,84.71180,83.51369,82.24278,81.05578,80.04140,79.24472,78.68060,78.34290,78.21252,78.26642,78.48735,78.87112,79.42636,80.16411,81.07893,82.12413,83.18207,84.03416,84.38428,84.05183,83.17019,82.04256,80.92727,79.98954,79.32227,78.96977,78.94528},
    {86.10744,86.22572,86.48778,86.87520,87.35811,87.88525,88.35363,88.55620,88.30834,87.72885,87.01271,86.26570,85.54319,84.87998,84.30054,83.82240,83.45756,83.21359,83.09470,83.10300,83.23944,83.50402,83.89510,84.40758,85.03003,85.74097,86.50311,87.25218,87.87193,88.17333,88.03502,87.60758,87.10856,86.66301,86.33143,86.14252,86.10744},
    {88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653,88.07653}
};

const float AP_Declination::intensity_table[19][37] = {
    {0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767,0.54767},
    {0.60825,0.60206,0.59434,0.58530,0.57515,0.56411,0.55241,0.54030,0.52809,0.51609,0.50461,0.49398,0.48448,0.47637,0.46990,0.46527,0.46269,0.46233,0.46433,0.46879,0.47569,0.48493,0.49627,0.50933,0.52363,0.53857,0.55354,0.56789,0.58105,0.59250,0.60185,0.60885,0.61338,0.61544,0.61515,0.61269,0.60825},
    {0.63245,0.61954,0.60490,0.58870,0.57104,0.55194,0.53154,0.51014,0.48830,0.46675,0.44634,0.42782,0.41172,0.39832,0.38768,0.37987,0.37509,0.37373,0.37639,0.38371,0.39616,0.41382,0.43634,0.46285,0.49213,0.52273,0.55305,0.58151,0.60666,0.62729,0.64263,0.65238,0.65671,0.65613,0.65139,0.64325,0.63245},
    {0.62084,0.60228,0.58272,0.56223,0.54051,0.51707,0.49146,0.46367,0.43448,0.40541,0.37843,0.35526,0.33684,0.32303,0.31294,0.30558,0.30060,0.29858,0.30090,0.30937,0.32552,0.34996,0.38210,0.42023,0.46210,0.50528,0.54736,0.58595,0.61880,0.64410,0.66087,0.66909,0.66955,0.66355,0.65253,0.63789,0.62084},
    {0.58610,0.56361,0.54099,0.51840,0.49544,0.47114,0.44427,0.41407,0.38113,0.34768,0.31706,0.29255,0.27594,0.26654,0.26163,0.25835,0.25527,0.25290,0.25335,0.25991,0.27610,0.30385,0.34238,0.38868,0.43886,0.48929,0.53702,0.57937,0.61377,0.63839,0.65267,0.65720,0.65336,0.64279,0.62708,0.60769,0.58610},
    {0.54046,0.51618,0.49216,0.46866,0.44557,0.42218,0.39704,0.36877,0.33724,0.30438,0.27416,0.25132,0.23895,0.23605,0.23815,0.24078,0.24192,0.24153,0.24082,0.24325,0.25467,0.27999,0.31977,0.36992,0.42414,0.47679,0.52421,0.56382,0.59346,0.61239,0.62158,0.62234,0.61580,0.60316,0.58557,0.56413,0.54046},
    {0.48860,0.46495,0.44155,0.41851,0.39611,0.37441,0.35270,0.32962,0.30405,0.27661,0.25074,0.23182,0.22379,0.22570,0.23264,0.24034,0.24752,0.25355,0.25687,0.25821,0.26340,0.28051,0.31403,0.36128,0.41409,0.46431,0.50720,0.53997,0.56069,0.57067,0.57362,0.57142,0.56379,0.55090,0.53335,0.51198,0.48860},
    {0.43249,0.41169,0.39127,0.37116,0.35176,0.33363,0.31693,0.30073,0.28327,0.26384,0.24478,0.23071,0.22538,0.22875,0.23748,0.24862,0.26162,0.27538,0.28590,0.29049,0.29235,0.29972,0.32093,0.35736,0.40163,0.44447,0.48024,0.50510,0.51647,0.51737,0.51452,0.51013,0.50214,0.48970,0.47324,0.45354,0.43249},
    {0.37917,0.36353,0.34855,0.33420,0.32086,0.30898,0.29892,0.29017,0.28098,0.26996,0.25783,0.24732,0.24160,0.24267,0.25042,0.26297,0.27857,0.29530,0.30921,0.31674,0.31839,0.31995,0.33021,0.35335,0.38472,0.41662,0.44365,0.46116,0.46574,0.46087,0.45421,0.44790,0.43911,0.42692,0.41213,0.39569,0.37917},
    {0.34151,0.33268,0.32460,0.31753,0.31208,0.30832,0.30605,0.30481,0.30305,0.29869,0.29095,0.28113,0.27213,0.26763,0.27054,0.28027,0.29360,0.30762,0.31982,0.32799,0.33145,0.33323,0.33955,0.35381,0.37362,0.39451,0.41268,0.42404,0.42552,0.41932,0.41077,0.40158,0.39039,0.37738,0.36410,0.35186,0.34151},
    {0.32870,0.32605,0.32443,0.32432,0.32680,0.33163,0.33769,0.34377,0.34786,0.34728,0.34058,0.32889,0.31551,0.30485,0.30092,0.30429,0.31219,0.32196,0.33195,0.34053,0.34691,0.35280,0.36098,0.37206,0.38479,0.39795,0.40977,0.41732,0.41834,0.41291,0.40249,0.38846,0.37242,0.35658,0.34315,0.33382,0.32870},
    {0.34034,0.34106,0.34423,0.35004,0.35942,0.37188,0.38550,0.39796,0.40648,0.40795,0.40069,0.38622,0.36889,0.35386,0.34495,0.34284,0.34586,0.35262,0.36189,0.37148,0.38028,0.38959,0.40025,0.41105,0.42127,0.43152,0.44130,0.44838,0.45042,0.44551,0.43247,0.41272,0.39037,0.36965,0.35353,0.34381,0.34034},
    {0.37289,0.37424,0.38039,0.39088,0.40549,0.42302,0.44122,0.45740,0.46855,0.47139,0.46399,0.44797,0.42831,0.41073,0.39890,0.39336,0.39307,0.39743,0.40550,0.41496,0.42431,0.43429,0.44554,0.45725,0.46886,0.48067,0.49213,0.50111,0.50477,0.50004,0.48509,0.46161,0.43465,0.40955,0.38994,0.37768,0.37289},
    {0.42307,0.42397,0.43133,0.44427,0.46134,0.48038,0.49897,0.51479,0.52541,0.52806,0.52104,0.50557,0.48601,0.46756,0.45370,0.44534,0.44204,0.44338,0.44858,0.45596,0.46417,0.47346,0.48473,0.49811,0.51308,0.52868,0.54332,0.55454,0.55939,0.55505,0.54027,0.51695,0.48975,0.46386,0.44306,0.42931,0.42307},
    {0.48388,0.48447,0.49107,0.50279,0.51789,0.53405,0.54895,0.56073,0.56771,0.56829,0.56154,0.54831,0.53139,0.51434,0.49996,0.48956,0.48345,0.48154,0.48336,0.48801,0.49471,0.50350,0.51505,0.52973,0.54695,0.56506,0.58166,0.59399,0.59941,0.59606,0.58369,0.56425,0.54149,0.51957,0.50171,0.48962,0.48388},
    {0.53980,0.54003,0.54407,0.55121,0.56026,0.56968,0.57796,0.58379,0.58622,0.58454,0.57851,0.56864,0.55625,0.54310,0.53087,0.52081,0.51367,0.50982,0.50931,0.51189,0.51733,0.52561,0.53688,0.55106,0.56739,0.58427,0.59948,0.61070,0.61602,0.61448,0.60643,0.59355,0.57844,0.56383,0.55186,0.54372,0.53980},
    {0.57275,0.57193,0.57268,0.57460,0.57713,0.57967,0.58158,0.58232,0.58148,0.57883,0.57432,0.56817,0.56086,0.55303,0.54542,0.53875,0.53364,0.53055,0.52980,0.53151,0.53573,0.54239,0.55127,0.56194,0.57366,0.58534,0.59569,0.60349,0.60783,0.60838,0.60543,0.59986,0.59293,0.58592,0.57985,0.57537,0.57275},
    {0.57801,0.57669,0.57561,0.57469,0.57383,0.57290,0.57180,0.57042,0.56868,0.56655,0.56406,0.56128,0.55835,0.55544,0.55276,0.55054,0.54900,0.54831,0.54861,0.54998,0.55241,0.55583,0.56006,0.56484,0.56985,0.57473,0.57911,0.58266,0.58518,0.58655,0.58683,0.58619,0.58488,0.58318,0.58134,0.57958,0.57801},
    {0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620,0.56620}
};

