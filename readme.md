
# goRuckig

`goRuckig` is a Go package that provides real-time motion planning capabilities using the Ruckig library. This package allows developers to calculate smooth and efficient trajectories with specified constraints, making it ideal for applications such as robotics, CNC machines, and pick-and-place systems.

## Features
- Real-time trajectory generation
- Support for multi-degree-of-freedom (DOF) systems
- Configurable velocity, acceleration, and jerk limits
- Efficient performance for high-speed applications

## Installation
Start by cloning the repo
```bash
git clone www.github.com/amken3d/goRuckig.git
cd goRuckig
git submodule update --init --recursive
```
The ruckig folder should now have the latest ruckig code
Edit the CMakeLists.txt file using your favorite editor and turn off these options
```
option(BUILD_EXAMPLES "Build example programs" OFF)
option(BUILD_PYTHON_MODULE "Build Python wrapper with nanobind" OFF)
option(BUILD_CLOUD_CLIENT "Build cloud client to calculate Ruckig Pro trajectories remotely" OFF)
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_BENCHMARK "Build benchmark" OFF)
option(BUILD_SHARED_LIBS "Build as shared library" OFF)
```
Save the file and run 

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
Check the build directory. You should now have a libruckig.a file
You can now move to using goRuckig in your Go project as described below.

### Using goRuckig

To use `goRuckig` in your Go project, first install the package using:
```bash
go get github.com/amken3d/goRuckig

```
Getting Started
Here's a quick example to get you started with goRuckig:

Example Code

```go
package main

import (
	"fmt"
	"github.com/amken3d/goRuckig"
)

func main() {
	// Create a Ruckig instance with a control cycle of 0.01s and 3 degrees of freedom
	ruckig := goRuckig.NewRuckig(0.01, 3)
	defer ruckig.Destroy()

	// Create input and output parameters
	input := goRuckig.NewInputParameter(3)
	defer input.Destroy()

	output := goRuckig.NewOutputParameter(3)
	defer output.Destroy()

	// Set the input parameters
	input.SetCurrentPosition([]float64{0.0, 0.0, 0.0})
	input.SetCurrentVelocity([]float64{0.0, 0.0, 0.0})
	input.SetCurrentAcceleration([]float64{0.0, 0.0, 0.0})

	input.SetTargetPosition([]float64{5.0, -2.0, -3.5})
	input.SetTargetVelocity([]float64{0.0, -0.5, -2.0})
	input.SetTargetAcceleration([]float64{0.0, 0.0, 0.5})

	input.SetMaxVelocity([]float64{3.0, 2.0, 1.0})
	input.SetMaxAcceleration([]float64{3.0, 2.0, 1.0})
	input.SetMaxJerk([]float64{4.0, 3.0, 2.0})

	// Generate the trajectory
	fmt.Println("t | position")
	for ruckig.Update(input, output) == 0 { // 0 indicates Result::Working
		fmt.Printf("%.2f | %v\n", output.Time(), output.NewPosition())
		output.PassToInput(input)
	}

	fmt.Printf("Trajectory duration: %.2f [s]\n", output.TrajectoryDuration())
}

```
## Usage
### Create an Instance: 
Use ```NewRuckig``` to create a Ruckig instance, specifying the control cycle and the number of degrees of freedom.
Configure Input Parameters: Set the current position, velocity, acceleration, and the desired target state. Also, configure the maximum velocity, acceleration, and jerk limits.
Generate Trajectory: Call Update in a loop to generate the trajectory, passing the output back to the input for each update.
Process Results: Use `output.NewPosition()` and `output.Time()` to get the position and time for each update.

## Edge Cases
Be sure to test different scenarios, such as:

Edge cases with zero initial and target velocities
Negative velocity cases
Extreme limits on velocity, acceleration, or jerk
These cases will help ensure your motion planning is robust and reliable.

#### Performance Testing

The package includes a performance test to measure how efficiently Ruckig can generate trajectory updates. The performance test results, such as 1447 updates in 514.9µs, demonstrate the high efficiency of the Ruckig library.

```go

=== RUN   TestRuckigTrajectory
t | position
0.01 | [6.666666666666667e-07 -0.021875500000000003 0.4949746666666667]
0.02 | [5.333333333333334e-06 -0.04350400000000001 0.48989733333333335]
0.03 | [1.7999999999999997e-05 -0.06488850000000002 0.484766]
0.04 | [4.266666666666667e-05 -0.08603200000000001 0.47957866666666665]
0.05 | [8.333333333333334e-05 -0.10693750000000002 0.47433333333333333]
0.06 | [0.00014400000000000003 -0.12760800000000003 0.469028]
0.07 | [0.00022866666666666673 -0.14804650000000003 0.46366066666666667]
0.08 | [0.00034133333333333335 -0.16825600000000002 0.4582293333333333]
0.09 | [0.00048599999999999994 -0.1882395 0.452732]
0.10 | [0.0006666666666666665 -0.208 0.44716666666666666]
0.11 | [0.000887333333333333 -0.22754049999999998 0.44153133333333333]
0.12 | [0.0011519999999999996 -0.246864 0.435824]
0.13 | [0.001464666666666666 -0.26597350000000003 0.43004276978900274]
0.14 | [0.0018293333333333328 -0.284872 0.4241877530418887]
0.15 | [0.0022499999999999994 -0.3035625 0.4182608835054543]
0.16 | [0.002730666666666667 -0.32204800000000006 0.4122641611796995]
0.17 | [0.0032753333333333337 -0.3403314814814815 0.4061995860646243]
0.18 | [0.0038880000000000013 -0.3584148148148149 0.40006915816022875]
0.19 | [0.0045726666666666685 -0.37629814814814827 0.3938748774665128]
0.20 | [0.005333333333333336 -0.3939814814814816 0.38761874398347645]
0.21 | [0.006174000000000005 -0.411464814814815 0.3813027577111197]
0.22 | [0.0070986666666666715 -0.4287481481481484 0.3749289186494426]
0.23 | [0.00811133333333334 -0.4458314814814818 0.3684992267984451]
0.24 | [0.009216000000000009 -0.46271481481481513 0.3620156821581272]
0.25 | [0.010416666666666673 -0.4793981481481485 0.35548028472848897]
0.26 | [0.011717333333333342 -0.4958814814814819 0.3488950345095303]
0.27 | [0.01312200000000001 -0.5121648148148154 0.3422619315012513]
0.28 | [0.01463466666666668 -0.5282481481481487 0.33558297570365186]
0.29 | [0.016259333333333348 -0.5441314814814822 0.32886016711673205]
0.30 | [0.01800000000000002 -0.5598148148148157 0.3220955057404919]
0.31 | [0.01986066666666669 -0.5752981481481491 0.3152909915749313]
0.32 | [0.021845333333333355 -0.5905814814814825 0.30844862462005035]
0.33 | [0.023958000000000028 -0.605664814814816 0.301570404875849]
0.34 | [0.026202666666666697 -0.6205481481481494 0.2946583323423273]
0.35 | [0.02858333333333337 -0.6352314814814829 0.28771440701948514]
0.36 | [0.03110400000000004 -0.6497148148148164 0.28074062890732265]
0.37 | [0.03376866666666671 -0.6639981481481498 0.2737389980058398]
0.38 | [0.03658133333333339 -0.6780814814814833 0.2667115143150365]
0.39 | [0.03954600000000005 -0.6919648148148168 0.25966017783491285]
0.40 | [0.04266666666666672 -0.7056481481481502 0.2525869885654688]
0.41 | [0.0459473333333334 -0.7191314814814838 0.2454939465067044]
0.42 | [0.049392000000000075 -0.7324148148148173 0.23838305165861956]
0.43 | [0.05300466666666675 -0.7454981481481509 0.23125630402121436]
0.44 | [0.05678933333333342 -0.7583814814814843 0.2241157035944888]
0.45 | [0.060750000000000096 -0.7710648148148178 0.21696325037844286]
0.46 | [0.06489066666666676 -0.7835481481481514 0.2098009443730765]
0.47 | [0.06921533333333345 -0.7958314814814849 0.2026307855783898]
0.48 | [0.07372800000000011 -0.8079148148148185 0.19545477399438266]
0.49 | [0.07843266666666679 -0.819798148148152 0.18827490962105517]
0.50 | [0.08333333333333345 -0.8314814814814855 0.18109319232548235]
0.51 | [0.08843400000000011 -0.8429648148148191 0.1739112100170949]
0.52 | [0.0937386666666668 -0.8542481481481526 0.16672922770870743]
0.53 | [0.09925133333333347 -0.8653314814814862 0.15954724540031998]
0.54 | [0.10497600000000015 -0.8762148148148198 0.15236526309193252]
0.55 | [0.11091666666666684 -0.8868981481481534 0.1451832807835451]
0.56 | [0.1170773333333335 -0.897381481481487 0.13800129847515763]
0.57 | [0.12346200000000018 -0.9076648148148206 0.13081931616677017]
0.58 | [0.13007466666666687 -0.9177481481481542 0.1236373338583827]
0.59 | [0.13691933333333353 -0.9276314814814878 0.11645535154999524]
0.60 | [0.1440000000000002 -0.937315023059528 0.10927336924160778]
0.61 | [0.1513206666666669 -0.9468008131390091 0.10209138693322033]
0.62 | [0.15888533333333357 -0.9560918436030806 0.09490940462483287]
0.63 | [0.16669800000000026 -0.9651911144517428 0.08772742231644541]
0.64 | [0.17476266666666695 -0.9741016256849954 0.08054544000805795]
0.65 | [0.18308333333333363 -0.9828263773028384 0.0733634576996705]
0.66 | [0.1916640000000003 -0.9913683693052721 0.06618147539128304]
0.67 | [0.200508666666667 -0.9997306016922961 0.05899949308289558]
0.68 | [0.2096213333333337 -1.0079160744639108 0.051817510774508135]
0.69 | [0.21900600000000037 -1.0159277876201158 0.04463552846612068]
0.70 | [0.22866666666666707 -1.0237687411609115 0.03745354615773322]
0.71 | [0.23860733333333375 -1.0314419350862976 0.03027156384934576]
0.72 | [0.24883200000000044 -1.0389503693962743 0.023089581540958304]
0.73 | [0.25934466666666717 -1.0462970440908415 0.015907599232570846]
0.74 | [0.2701493333333338 -1.053484959169999 0.008725616924183388]
0.75 | [0.2812500000000005 -1.060517114633747 0.0015436346157959302]
0.76 | [0.2926500000000005 -1.0673965104820857 -0.005638347692591528]
0.77 | [0.30435000000000056 -1.074126146715015 -0.012820330000978986]
0.78 | [0.3163500000000006 -1.0807090233325345 -0.020002312309366443]
0.79 | [0.3286500000000006 -1.0871481403346446 -0.0271842946177539]
0.80 | [0.3412500000000006 -1.0934464977213454 -0.03436627692614136]
0.81 | [0.35415000000000063 -1.0996070954926365 -0.04154825923452882]
0.82 | [0.3673500000000007 -1.105632933648518 -0.048730241542916275]
0.83 | [0.3808500000000007 -1.1115270121889902 -0.05591222385130373]
0.84 | [0.3946500000000007 -1.117292331114053 -0.06309420615969119]
0.85 | [0.4087500000000007 -1.122931890423706 -0.07027618846807865]
0.86 | [0.4231500000000008 -1.1284486901179498 -0.0774581707764661]
0.87 | [0.43785000000000085 -1.133845730196784 -0.08464015308485356]
0.88 | [0.45285000000000086 -1.1391260106602086 -0.09182213539324102]
0.89 | [0.46815000000000084 -1.1442925315082237 -0.09900411770162848]
0.90 | [0.4837500000000009 -1.1493482927408294 -0.10618610001001594]
0.91 | [0.4996500000000009 -1.1542962943580255 -0.1133680823184034]
0.92 | [0.5158500000000009 -1.1591395363598123 -0.12055006462679085]
0.93 | [0.532350000000001 -1.1638810187461894 -0.1277320469351783]
0.94 | [0.5491500000000011 -1.1685237415171572 -0.13491402924356577]
0.95 | [0.566250000000001 -1.1730707046727153 -0.14209601155195323]
0.96 | [0.5836500000000011 -1.177524908212864 -0.14927799386034069]
0.97 | [0.6013499998215126 -1.1818893521376033 -0.15645997616872814]
0.98 | [0.6193491959435702 -1.186167036446933 -0.1636419584771156]
0.99 | [0.6376441342591245 -1.1903609611408532 -0.17082394078550306]
1.00 | [0.6562308147681756 -1.1944741262193639 -0.17800592309389052]
1.01 | [0.6751052374707233 -1.1985095316824652 -0.18518790540227797]
1.02 | [0.6942634023667678 -1.2024701775301567 -0.19236988771066543]
1.03 | [0.7137013094563089 -1.2063590637624388 -0.1995518700190529]
1.04 | [0.7334149587393469 -1.2101791903793118 -0.20673385232744035]
1.05 | [0.7534003502158816 -1.213933557380775 -0.2139158346358278]
1.06 | [0.7736534838859129 -1.2176251647668286 -0.22109781694421526]
1.07 | [0.794170359749441 -1.2212570125374729 -0.22827979925260272]
1.08 | [0.8149469778064659 -1.2248321006927076 -0.23546178156099018]
1.09 | [0.8359793380569873 -1.2283534292325329 -0.24264376386937764]
1.10 | [0.8572634405010056 -1.2318239981569485 -0.2498257461777651]
1.11 | [0.8787952851385206 -1.2352468074659548 -0.25700772848615255]
1.12 | [0.9005708719695323 -1.2386248571595515 -0.26418971079454]
1.13 | [0.9225862009940408 -1.2419611472377388 -0.27137169310292747]
1.14 | [0.9448372722120459 -1.2452586777005166 -0.2785536754113149]
1.15 | [0.9673200856235478 -1.248520448547885 -0.2857356577197024]
1.16 | [0.9900306412285464 -1.2517494597798435 -0.29291764002808984]
1.17 | [1.0129649390270417 -1.2549487113963929 -0.3000996223364773]
1.18 | [1.0361189790190337 -1.2581212033975326 -0.30728160464486476]
1.19 | [1.0594887612045225 -1.2612699357832629 -0.3144635869532522]
1.20 | [1.083070285583508 -1.2643979085535837 -0.3216455692616397]
1.21 | [1.1068595521559903 -1.267508121708495 -0.3288275515700271]
1.22 | [1.1308525609219693 -1.2706035752479967 -0.3360095338784146]
1.23 | [1.1550453118814448 -1.273687269172089 -0.343191516186802]
1.24 | [1.1794338050344173 -1.2767622034807717 -0.3503734984951895]
1.25 | [1.2040140403808863 -1.2798313781740451 -0.3575554808035769]
1.26 | [1.2287820179208522 -1.2828977929946763 -0.3647374631119644]
1.27 | [1.2537337376543147 -1.2859638186340434 -0.3719194454203518]
1.28 | [1.2788651995812739 -1.2890298442734105 -0.37910142772873934]
1.29 | [1.3041724037017297 -1.2920958699127778 -0.38628341003712674]
1.30 | [1.3296513500156826 -1.295161895552145 -0.39346539234551425]
1.31 | [1.3552980385231321 -1.298227921191512 -0.40064737465390166]
1.32 | [1.381108469224078 -1.3012939468308793 -0.40782935696228917]
1.33 | [1.407078642118521 -1.3043599724702464 -0.41501133927067657]
1.34 | [1.433204557206461 -1.3074259981096135 -0.4221933215790641]
1.35 | [1.4594822144878972 -1.3104920237489806 -0.4293753621381323]
1.36 | [1.48590761396283 -1.3135580493883479 -0.43655854943065486]
1.37 | [1.5124767556312597 -1.316624075027715 -0.4437448548835129]
1.38 | [1.5391856394931864 -1.319690100667082 -0.45093627849670653]
1.39 | [1.5660302655486096 -1.3227561263064493 -0.45813482027023567]
1.40 | [1.5930066337975295 -1.3258221519458164 -0.4653424802041003]
1.41 | [1.6201107442399463 -1.3288881775851835 -0.4725612582983005]
1.42 | [1.6473385968758596 -1.3319542032245506 -0.4797931545528362]
1.43 | [1.6746861917052698 -1.335020228863918 -0.48704016896770747]
1.44 | [1.7021495287281765 -1.338086254503285 -0.4943043015429142]
1.45 | [1.72972460794458 -1.341152280142652 -0.5015875522784565]
1.46 | [1.7574074293544804 -1.3442183057820194 -0.5088919211743343]
1.47 | [1.7851939929578775 -1.3472843314213865 -0.5162194082305477]
1.48 | [1.8130802987547712 -1.3503503570607536 -0.5235720134470966]
1.49 | [1.8410623467451614 -1.3534163827001207 -0.530951736823981]
1.50 | [1.8691361369290487 -1.356482408339488 -0.5383605783612009]
1.51 | [1.8972976693064325 -1.359548433978855 -0.5458005380587564]
1.52 | [1.9255429438773133 -1.3626144596182221 -0.5532736159166474]
1.53 | [1.9538679606416904 -1.3656804852575894 -0.560781811934874]
1.54 | [1.9822687195995645 -1.3687465108969565 -0.5683271261134359]
1.55 | [2.010741220750935 -1.3718125365363236 -0.5759115584523335]
1.56 | [2.039281464095803 -1.374878562175691 -0.5835371089515666]
1.57 | [2.067885449634167 -1.377944587815058 -0.5912057776111354]
1.58 | [2.096549177366028 -1.381010613454425 -0.5989195644310394]
1.59 | [2.1252686472913855 -1.3840766390937922 -0.6066804694112792]
1.60 | [2.15403985941024 -1.3871426647331595 -0.6144904925518544]
1.61 | [2.182858813722591 -1.3902086903725266 -0.6223516338527652]
1.62 | [2.211721510228439 -1.3932747160118937 -0.6302658933140115]
1.63 | [2.2406239489277837 -1.396340741651261 -0.6382352709355933]
1.64 | [2.269562129820625 -1.399406767290628 -0.6462617667175106]
1.65 | [2.298532052906963 -1.4024727929299952 -0.6543473806597635]
1.66 | [2.3275297181867978 -1.4055388185693625 -0.6624941127623518]
1.67 | [2.356551125660129 -1.4086048442087296 -0.6707039630252758]
1.68 | [2.3855922753269567 -1.4116708698480966 -0.6789789314485353]
1.69 | [2.414649167187282 -1.4147368954874637 -0.6873210180321303]
1.70 | [2.4437178012411036 -1.417802921126831 -0.6957322227760607]
1.71 | [2.472794177488422 -1.4208689467661981 -0.7042145456803268]
1.72 | [2.501874295929237 -1.4239349724055652 -0.7127699867449284]
1.73 | [2.5309541565635487 -1.4270009980449325 -0.7214005459698655]
1.74 | [2.560029759391357 -1.4300670236842996 -0.7301082233551381]
1.75 | [2.5890971044126623 -1.4331330493236667 -0.7388950189007463]
1.76 | [2.6181521916274644 -1.4361990749630338 -0.7477629326066899]
1.77 | [2.6471910210357628 -1.439265100602401 -0.7567139644729692]
1.78 | [2.676209592637558 -1.4423311262417682 -0.765750114499584]
1.79 | [2.7052039064328506 -1.4453971518811353 -0.7748733826865342]
1.80 | [2.7341699624216393 -1.4484631775205026 -0.78408576903382]
1.81 | [2.763103760603925 -1.4515292031598697 -0.7933892735414414]
1.82 | [2.792001300979707 -1.4545952287992368 -0.8027858962093981]
1.83 | [2.8208585835489863 -1.4576612544386038 -0.8122776370376905]
1.84 | [2.8496716083117617 -1.4607272800779711 -0.8218664960263184]
1.85 | [2.878436375268034 -1.4637933057173382 -0.831554414924601]
1.86 | [2.9071488844178033 -1.4668593313567053 -0.8413423052497649]
1.87 | [2.9358051357610693 -1.4699253569960726 -0.8512301955749288]
1.88 | [2.964401129297832 -1.4729913826354397 -0.8612180859000927]
1.89 | [2.992932865028091 -1.4760574082748068 -0.8713059762252566]
1.90 | [3.021396342951847 -1.479123433914174 -0.8814938665504205]
1.91 | [3.0497875630691 -1.4821894595535412 -0.8917817568755844]
1.92 | [3.0781025253798493 -1.4852554851929083 -0.9021696472007482]
1.93 | [3.1063372298840957 -1.4883215108322754 -0.9126575375259122]
1.94 | [3.1344876765818386 -1.4913875364716427 -0.923245427851076]
1.95 | [3.1625498654730784 -1.4944535621110098 -0.9339333181762399]
1.96 | [3.1905197965578145 -1.4975195877503769 -0.9447212085014038]
1.97 | [3.218393469836048 -1.500585613389744 -0.9556090988265677]
1.98 | [3.246166885307778 -1.5036516390291113 -0.9665969891517316]
1.99 | [3.273836042973004 -1.5067176646684783 -0.9776848794768955]
2.00 | [3.301396942831727 -1.5097836903078454 -0.9888727698020591]
2.01 | [3.328845584883946 -1.5128497159472125 -1.0001606601272228]
2.02 | [3.3561779691296625 -1.5159157415865796 -1.0115485504523865]
2.03 | [3.383390095568875 -1.5189817672259467 -1.02303644077755]
2.04 | [3.4104779642015846 -1.5220477928653138 -1.0346243311027137]
2.05 | [3.437437575027791 -1.5251138185046809 -1.0463122214278773]
2.06 | [3.464264928047494 -1.528179844144048 -1.0581001117530409]
2.07 | [3.4909560232606935 -1.531245869783415 -1.0699880020782044]
2.08 | [3.5175068606673903 -1.5343118954227821 -1.081975892403368]
2.09 | [3.543913440267583 -1.5373779210621492 -1.0940637827285316]
2.10 | [3.5701717620612734 -1.5404439467015163 -1.1062516730536953]
2.11 | [3.5962778260484605 -1.5435099723408834 -1.1185395633788588]
2.12 | [3.6222276322291433 -1.5465759979802505 -1.1309274537040226]
2.13 | [3.6480171806033237 -1.5496420236196176 -1.143415344029186]
2.14 | [3.6736424711710005 -1.5527080492589846 -1.1560032343543496]
2.15 | [3.699099503932174 -1.5557740748983517 -1.1686911246795133]
2.16 | [3.7243842788868444 -1.5588401005377188 -1.1814790150046768]
2.17 | [3.7494927960350113 -1.561906126177086 -1.1943669053298405]
2.18 | [3.774421055376675 -1.564972151816453 -1.207354795655004]
2.19 | [3.7991650569118356 -1.56803817745582 -1.2204426859801676]
2.20 | [3.8237208006404932 -1.5711042030951872 -1.233630576305331]
2.21 | [3.848084286562647 -1.5741702287345543 -1.2469184666304947]
2.22 | [3.8722515146782976 -1.5772362543739213 -1.260306356955658]
2.23 | [3.896218484987445 -1.5803022800132884 -1.2737942472808217]
2.24 | [3.919981197490089 -1.5833683056526555 -1.2873821376059853]
2.25 | [3.94353565218623 -1.5864343312920226 -1.3010700279311487]
2.26 | [3.9668778490758676 -1.5895003569313895 -1.3148579182563123]
2.27 | [3.9900037881590023 -1.5925663825707566 -1.3287458085814756]
2.28 | [4.012909469435633 -1.5956324082101236 -1.3427336989066392]
2.29 | [4.035590892905761 -1.5986984338494907 -1.356821589231803]
2.30 | [4.0580440585693855 -1.6017644594888578 -1.3710094795569663]
2.31 | [4.080264966426507 -1.604830485128225 -1.3852973698821298]
2.32 | [4.102249616477125 -1.607896510767592 -1.3996852602072933]
2.33 | [4.12399400872124 -1.610962536406959 -1.4141731505324568]
2.34 | [4.145494143158851 -1.6140285620463262 -1.4287610408576203]
2.35 | [4.16674601978996 -1.6170945876856933 -1.4434489311827838]
2.36 | [4.187745638614564 -1.6201606133250603 -1.4582368215079473]
2.37 | [4.208488999632666 -1.6232266389644274 -1.4731247118331108]
2.38 | [4.228972102844265 -1.6262926646037945 -1.4881126021582742]
2.39 | [4.2491909482493595 -1.6293586902431616 -1.5032004924834377]
2.40 | [4.269141535847952 -1.6324247158825287 -1.5183883828086011]
2.41 | [4.288819865640041 -1.6354907415218958 -1.5336762731337648]
2.42 | [4.308221937625626 -1.6385567671612629 -1.5490641634589282]
2.43 | [4.327343751804708 -1.64162279280063 -1.5645520537840916]
2.44 | [4.346181308177287 -1.644688818439997 -1.5801399441092552]
2.45 | [4.364730606743363 -1.6477548440793641 -1.5958278344344183]
2.46 | [4.382987647502935 -1.6508208697187312 -1.611615724759582]
2.47 | [4.400948430634492 -1.6538868953580983 -1.6275036150847453]
2.48 | [4.418609759659001 -1.6569529209974654 -1.6434915054099088]
2.49 | [4.435971088683509 -1.6600189466368325 -1.6595793957350722]
2.50 | [4.453032417708017 -1.6630849722761996 -1.6757672860602355]
2.51 | [4.469793746732526 -1.6661509979155666 -1.692055176385399]
2.52 | [4.486255075757033 -1.6692170235549337 -1.7084430667105623]
2.53 | [4.502416404781542 -1.6722830491943008 -1.7249309570357259]
2.54 | [4.51827773380605 -1.675349074833668 -1.7415188473608894]
2.55 | [4.533839062830559 -1.678415100473035 -1.7582067376860526]
2.56 | [4.549100391855068 -1.681481126112402 -1.774994628011216]
2.57 | [4.564061720879575 -1.6845471517517692 -1.7918825183363796]
2.58 | [4.578723049904084 -1.6876131773911363 -1.808870408661543]
2.59 | [4.593084378928593 -1.6906792030305033 -1.8259582989867065]
2.60 | [4.607145707953101 -1.6937452286698704 -1.8431461893118697]
2.61 | [4.620907036977609 -1.6968112543092375 -1.8604340796370331]
2.62 | [4.634368366002118 -1.6998772799486046 -1.8778219699621965]
2.63 | [4.647529695026626 -1.7029433055879717 -1.89530986028736]
2.64 | [4.660391024051135 -1.7060093312273388 -1.912897750612523]
2.65 | [4.672952353075643 -1.7090753568667059 -1.9305856409376865]
2.66 | [4.6852136821001515 -1.712141382506073 -1.9483735312628498]
2.67 | [4.69717501112466 -1.71520740814544 -1.9662614215880132]
2.68 | [4.708836340149169 -1.7182734337848071 -1.9842493119131765]
2.69 | [4.720197670601586 -1.7213394594241742 -2.002337201524386]
2.70 | [4.7312599573313605 -1.7244054850635413 -2.020524612996916]
2.71 | [4.742026759674142 -1.7274715107029084 -2.0388097666629426]
2.72 | [4.752502077629931 -1.7305375363422755 -2.057190662522466]
2.73 | [4.762689911198725 -1.7336035619816426 -2.0756653005754866]
2.74 | [4.772594260380527 -1.7366695876210096 -2.0942316808220034]
2.75 | [4.7822191251753345 -1.7397356132603767 -2.112887803262017]
2.76 | [4.791568505583149 -1.7428016388997438 -2.1316316678955274]
2.77 | [4.80064640160397 -1.745867664539111 -2.150461274722535]
2.78 | [4.809456813237798 -1.748933690178478 -2.1693746237430385]
2.79 | [4.818003740484632 -1.751999715817845 -2.188369714957039]
2.80 | [4.826291183344472 -1.7550657414572122 -2.2074445483645366]
2.81 | [4.83432314181732 -1.7581317670965793 -2.2265971239655307]
2.82 | [4.842103615903174 -1.7611977927359463 -2.2458254417600214]
2.83 | [4.849636605602034 -1.7642638183753134 -2.2651275017480086]
2.84 | [4.856926110913901 -1.7673298440146805 -2.284501303929493]
2.85 | [4.863976131838775 -1.7703958696540476 -2.3039448483044738]
2.86 | [4.870790668376654 -1.7734618952934147 -2.3234561348729517]
2.87 | [4.877373720527541 -1.7765279209327818 -2.3430331636349258]
2.88 | [4.883729288291434 -1.7795939465721489 -2.362673934590397]
2.89 | [4.889861371668334 -1.782659972211516 -2.3823764477393645]
2.90 | [4.89577397065824 -1.785725997850883 -2.402138703081829]
2.91 | [4.901471085261153 -1.7887920234902501 -2.4219587006177905]
2.92 | [4.906956715477072 -1.7918580491296172 -2.4418344403472485]
2.93 | [4.912234861305998 -1.7949240747689843 -2.4617639222702032]
2.94 | [4.917309522747931 -1.7979904761270586 -2.481745146386655]
2.95 | [4.9221846998028695 -1.8010596052812597 -2.5017761126966027]
2.96 | [4.926864392470815 -1.8041344618565425 -2.521854821200048]
2.97 | [4.931352600751768 -1.807218045852907 -2.5419792718969894]
2.98 | [4.935653324645726 -1.8103133572703536 -2.5621474647874276]
2.99 | [4.939770564152691 -1.813423396108882 -2.5823573998713627]
3.00 | [4.943708319272663 -1.816551162368492 -2.6026070771487944]
3.01 | [4.947470590005642 -1.8196996560491838 -2.6228944966197227]
3.02 | [4.9510613763516265 -1.8228718771509578 -2.643217658284148]
3.03 | [4.954484678310618 -1.8260708256738134 -2.6635745621420703]
3.04 | [4.957744495882616 -1.829299501617751 -2.683963208193489]
3.05 | [4.960844829067621 -1.8325609049827702 -2.704381596438404]
3.06 | [4.963789677865631 -1.8358580357688714 -2.7248277268768164]
3.07 | [4.96658304227665 -1.8391938939760544 -2.7452995995087255]
3.08 | [4.969228922300673 -1.842571479604319 -2.7657952143341307]
3.09 | [4.971731317937705 -1.8459937926536658 -2.786312571353033]
3.10 | [4.974094229187742 -1.8494638331240942 -2.806849670565432]
3.11 | [4.976321656050786 -1.8529846010156046 -2.827404511971328]
3.12 | [4.978417598526836 -1.8565590963281968 -2.8479750955707206]
3.13 | [4.9803860566158935 -1.8601903190618707 -2.8685594213636096]
3.14 | [4.982231030317957 -1.8638812692166264 -2.8891554893499958]
3.15 | [4.983956519633027 -1.8676349467924642 -2.9097612995298783]
3.16 | [4.985566524561103 -1.8714543517893838 -2.9303748519032577]
3.17 | [4.987065045102187 -1.875342484207385 -2.950994146470134]
3.18 | [4.988456081256277 -1.8793023440464682 -2.9716171832305065]
3.19 | [4.989743633023373 -1.8833367914906523 -2.992241962184376]
3.20 | [4.9909317004034754 -1.8874457409555752 -3.0128664833317425]
3.21 | [4.992024283396585 -1.891626303710743 -3.0334887466726057]
3.22 | [4.993025382002701 -1.895875479756156 -3.054106752206965]
3.23 | [4.993938996221823 -1.900190269091814 -3.074718499934822]
3.24 | [4.994769126053953 -1.904567671717717 -3.095321989856175]
3.25 | [4.995519771499088 -1.909004687633865 -3.115915221971025]
3.26 | [4.996194932557231 -1.9134983168402584 -3.1364961962793716]
3.27 | [4.996798609228379 -1.9180455593368968 -3.157062912781215]
3.28 | [4.997334801512534 -1.9226434151237803 -3.1776133714765553]
3.29 | [4.997807509409696 -1.9272888842009088 -3.1981455723653918]
3.30 | [4.998220732919864 -1.9319789665682825 -3.2186575154477257]
3.31 | [4.9985784720430395 -1.9367106622259014 -3.239147200723556]
3.32 | [4.99888472677922 -1.941480971173765 -3.259612628192883]
3.33 | [4.999143497128409 -1.946286893411874 -3.280051797855707]
3.34 | [4.999358783090603 -1.951125428940228 -3.3004627097120274]
3.35 | [4.999534584665804 -1.955993577758827 -3.3208433637618446]
3.36 | [4.999674901854012 -1.9608883398676713 -3.3411917600051586]
3.37 | [4.999783734655226 -1.9658067152667607 -3.361505898441969]
3.38 | [4.999865083069446 -1.970745703956095 -3.3817837790722765]
3.39 | [4.999922947096674 -1.9757023059356744 -3.4020234018960807]
3.40 | [4.999961326736908 -1.980673521205499 -3.422222766913382]
3.41 | [4.999984221990148 -1.9856563497655686 -3.442379874124179]
3.42 | [4.999995632856395 -1.9906477916158833 -3.4624927235284737]
3.43 | [4.999999559335649 -1.9956448467564432 -3.482559315126265]
Trajectory duration: 3.44 [s]
--- PASS: TestRuckigTrajectory (0.00s)
=== RUN   TestRuckigEdgeCases
=== RUN   TestRuckigEdgeCases/Basic_edge_case_with_zero_initial_and_target_velocities
Trajectory duration: 102.00 [s]
--- PASS: TestRuckigEdgeCases/Basic_edge_case_with_zero_initial_and_target_velocities (0.00s)
=== RUN   TestRuckigEdgeCases/Edge_case_with_negative_velocities
Trajectory duration: 19.83 [s]
--- PASS: TestRuckigEdgeCases/Edge_case_with_negative_velocities (0.00s)
=== RUN   TestRuckigEdgeCases/Case_with_extreme_jerk_limits
Trajectory duration: 1.91 [s]
--- PASS: TestRuckigEdgeCases/Case_with_extreme_jerk_limits (0.00s)
--- PASS: TestRuckigEdgeCases (0.00s)
=== RUN   TestRuckigAdditionalEdgeCases
=== RUN   TestRuckigAdditionalEdgeCases/High_Initial_Velocity_and_Low_Target_Velocity
Trajectory duration: 6.40 [s]
--- PASS: TestRuckigAdditionalEdgeCases/High_Initial_Velocity_and_Low_Target_Velocity (0.00s)
=== RUN   TestRuckigAdditionalEdgeCases/Low_Jerk_Limits
Trajectory duration: 6.84 [s]
--- PASS: TestRuckigAdditionalEdgeCases/Low_Jerk_Limits (0.00s)
=== RUN   TestRuckigAdditionalEdgeCases/Non-Uniform_Constraints_Across_Degrees_of_Freedom
Trajectory duration: 6.84 [s]
--- PASS: TestRuckigAdditionalEdgeCases/Non-Uniform_Constraints_Across_Degrees_of_Freedom (0.00s)
=== RUN   TestRuckigAdditionalEdgeCases/Rapid_Changes_in_Target_Position
Trajectory duration: 6.84 [s]
--- PASS: TestRuckigAdditionalEdgeCases/Rapid_Changes_in_Target_Position (0.00s)
--- PASS: TestRuckigAdditionalEdgeCases (0.00s)
=== RUN   TestRuckigPerformance
Performance test: 1447 updates in 514.9µs
--- PASS: TestRuckigPerformance (0.00s)
PASS

Process finished with the exit code 0

```
