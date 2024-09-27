#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from tf2_msgs.msg import TFMessage
import math
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from dataclasses import dataclass

@dataclass
class RobotTFWrapper:
    robot_name: str
    tf_message: TFMessage

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        Velocity = {'box_bot0': 1.22961467, 'box_bot1': 1.88371221, 'box_bot2': 1.62548941, 'box_bot3': 1.17236986, 
                    'box_bot4': 1.32454697, 'box_bot5': 1.67904082, 'box_bot6': 1.6253019, 'box_bot7': 1.13006307, 
                    'box_bot8': 1.19242935}
        Energy = {'box_bot0': {'locations': [(-3, -2, 1)], 'times': [0], 'energy': [0]},
                  'box_bot1': {'locations': [(-3, -1, 1)], 'times': [0], 'energy': [0]}, 
                  'box_bot2': {'locations': [(-3, 0, 1)], 'times': [0], 'energy': [0]}, 
                  'box_bot3': {'locations': [(-3, 1, 1), (4, 4, 1), (4, 4, 1), (7, 1, 1), (7, 1, 1), (7, 3, 1), (7, 3, 1), (9, 4, 1), (9, 4, 1), (9, 0, 1), (9, 0, 1), (9, 8, 1), (9, 8, 1), (9, 9, 1), (9, 9, 1), (8, 9, 1), (8, 9, 1), (-3, 1, 1), (-3, 1, 1), (7, 9, 1), (7, 9, 1), (-3, 1, 1), (-3, 1, 1)], 'times': [0, 6.641023693143965, 7.250616455911117, 10.869475014137205, 11.479067776904357, 13.18501406135545, 13.794606824122601, 15.701912753120519, 16.31150551588767, 19.723398084789856, 20.332990847557006, 27.156775985361378, 27.766368748128528, 28.619341890354075, 29.228934653121225, 30.081907795346773, 30.691500558113923, 41.557487773100405, 43.322641854531966, 54.855620619026425, 55.465213381793575, 65.77900662075373, 66.18247877549797], 'energy': [0, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.008020254689529376, 0, 0.0, 0.011354998398906564, 0.007634327674568865, 0]}, 
                  'box_bot4': {'locations': [(-3, 2, 1)], 'times': [0], 'energy': [0]}, 
                  'box_bot5': {'locations': [(-3, 3, 1)], 'times': [0], 'energy': [0]}, 
                  'box_bot6': {'locations': [(-3, 4, 1), (0, 0, 1), (0, 0, 1), (0, 1, 1), (0, 1, 1), (1, 0, 1), (1, 0, 1), (3, 0, 1), (3, 0, 1), (4, 0, 1), (4, 0, 1), (5, 1, 1), (5, 1, 1), (5, 2, 1), (5, 2, 1), (3, 2, 1), (3, 2, 1), (2, 2, 1), (2, 2, 1), (2, 3, 1), (2, 3, 1), (1, 3, 1), (1, 3, 1), (0, 3, 1), (0, 3, 1), (1, 4, 1), (1, 4, 1), (1, 5, 1), (1, 5, 1), (1, 6, 1), (1, 6, 1), (0, 6, 1), (0, 6, 1), (0, 7, 1), (0, 7, 1), (1, 7, 1), (1, 7, 1), (2, 7, 1), (2, 7, 1), (2, 6, 1), (2, 6, 1), (3, 6, 1), (3, 6, 1), (3, 7, 1), (3, 7, 1), (4, 7, 1), (4, 7, 1), (5, 7, 1), (5, 7, 1), (5, 6, 1), (5, 6, 1), (6, 6, 1), (6, 6, 1), (6, 7, 1), (6, 7, 1), (6, 8, 1), (6, 8, 1), (5, 8, 1), (5, 8, 1), (5, 9, 1), (5, 9, 1), (3, 9, 1), (3, 9, 1), (2, 8, 1), (2, 8, 1), (1, 9, 1), (1, 9, 1), (0, 9, 1), (0, 9, 1), (-3, 4, 1), (-3, 4, 1)], 'times': [0, 1.792069213934551, 2.7140148166953955, 3.32928512262685, 4.251230725387694, 5.121354336561401, 6.043299939322245, 7.273840551185155, 8.195786153945999, 8.811056459877454, 9.733002062638299, 10.603125673812006, 11.52507127657285, 12.140341582504306, 13.06228718526515, 14.292827797128059, 15.214773399888903, 15.830043705820358, 16.7519893085812, 17.36725961451266, 18.2892052172735, 18.904475523204958, 19.8264211259658, 20.441691431897258, 21.3636370346581, 22.233760645831808, 23.15570624859265, 23.770976554524108, 24.69292215728495, 25.308192463216407, 26.23013806597725, 26.845408371908707, 27.76735397466955, 28.382624280601007, 29.30456988336185, 29.919840189293307, 30.84178579205415, 31.457056097985607, 32.37900170074645, 32.9942720066779, 33.916217609438746, 34.5314879153702, 35.45343351813104, 36.068703824062496, 36.99064942682334, 37.60591973275479, 38.527865335515635, 39.14313564144709, 40.06508124420793, 40.680351550139385, 41.60229715290023, 42.21756745883168, 43.139513061592524, 43.75478336752398, 44.67672897028482, 45.29199927621627, 46.21394487897712, 46.82921518490857, 47.75116078766941, 48.366431093600866, 49.28837669636171, 50.51891730822462, 51.440862910985466, 52.310986522159176, 53.23293212492002, 54.10305573609373, 55.02500133885457, 55.640271644786026, 56.56221724754687, 61.823661692766144, 64.99603910723965], 'energy': [0, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.033199926137435366, 0]}, 
                  'box_bot7': {'locations': [(-3, 5, 1)], 'times': [0], 'energy': [0]}, 
                  'box_bot8': {'locations': [(-3, 6, 1)], 'times': [0], 'energy': [0]}}
        Ours = {'box_bot0': {'locations': [(-3, -2, 1), (1, 3, 1), (1, 3, 1), (1, 4, 1), (1, 4, 1), (2, 6, 1), (2, 6, 1), (3, 6, 1), (3, 6, 1), (3, 9, 1), (3, 9, 1), (-3, -2, 1), (-3, -2, 1)], 'times': [0, 4.341605824578585, 5.046189351128035, 5.859452260535599, 6.564035787085049, 8.382546936099617, 9.087130462649068, 9.900393372056632, 10.604976898606083, 13.044765626828775, 13.749349153378226, 21.80387522420764, 23.331641274081115], 'energy': [0, 0.0, 0.008928659092230019, 0.0, 0.008928659092230019, 0.0, 0.008928659092230019, 0.0, 0.008928659092230019, 0.0, 0.008928659092230019, 0.045247535470455486, 0]}, 
                'box_bot1': {'locations': [(-3, -1, 1), (5, 6, 1), (5, 6, 1), (4, 7, 1), (4, 7, 1), (5, 7, 1), (5, 7, 1), (5, 8, 1), (5, 8, 1), (5, 9, 1), (5, 9, 1), (6, 8, 1), (6, 8, 1), (7, 9, 1), (7, 9, 1), (-3, -1, 1), (-3, -1, 1)], 'times': [0, 5.691132429338573, 6.4879161230379685, 7.2386749477268975, 8.035458641426292, 8.566325297399477, 9.363108991098871, 9.893975647072056, 10.69075934077145, 11.221625996744635, 12.01840969044403, 12.769168515132959, 13.565952208832353, 14.316711033521283, 15.113494727220678, 21.115121336934923, 32.494845431779396], 'energy': [0, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.13051368598287016, 0]}, 
                'box_bot2': {'locations': [(-3, 0, 1), (0, 0, 1), (0, 0, 1), (1, 0, 1), (1, 0, 1), (3, 0, 1), (3, 0, 1), (5, 2, 1), (5, 2, 1), (7, 1, 1), (7, 1, 1), (6, 7, 1), (6, 7, 1), (8, 9, 1), (8, 9, 1), (-3, 0, 1), (-3, 0, 1)], 'times': [0, 1.4336142162838172, 1.9972051927078385, 2.6124045254025905, 3.1759955018266117, 4.406394167216115, 4.9699851436401365, 6.710031623359729, 7.27362259978375, 8.649250127401725, 9.212841103825745, 12.954952553405851, 13.518543529829874, 15.258590009549465, 15.822180985973485, 23.53525054563528, 25.271104334195492], 'energy': [0, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.018876323937967947, 0]}, 
                'box_bot3': {'locations': [(-3, 1, 1), (0, 3, 1), (0, 3, 1), (1, 5, 1), (1, 5, 1), (1, 6, 1), (1, 6, 1), (1, 7, 1), (1, 7, 1), (2, 7, 1), (2, 7, 1), (-3, 1, 1), (-3, 1, 1)], 'times': [0, 4.126491123978075, 4.736083886745226, 6.643389815743144, 7.252982578510296, 8.105955720735842, 8.715548483502994, 9.56852162572854, 10.178114388495692, 11.031087530721237, 11.640680293488389, 18.318893252551568, 19.341041163764146], 'energy': [0, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.005093429537347917, 0]}, 
                'box_bot4': {'locations': [(-3, 2, 1), (4, 0, 1), (4, 0, 1), (5, 1, 1), (5, 1, 1), (7, 3, 1), (7, 3, 1), (6, 6, 1), (6, 6, 1), (9, 8, 1), (9, 8, 1), (-3, 2, 1), (-3, 2, 1)], 'times': [0, 4.556472387477697, 5.2633119437682305, 6.331007987195533, 7.037847543486066, 9.173239630340671, 9.880079186631203, 12.267520119042219, 12.974359675332753, 15.696461155343563, 16.403300711634095, 25.853612484535404, 29.888877022590844], 'energy': [0, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.11096787293859779, 0]}, 
                'box_bot5': {'locations': [(-3, 3, 1), (2, 3, 1), (2, 3, 1), (3, 2, 1), (3, 2, 1), (4, 4, 1), (4, 4, 1), (3, 7, 1), (3, 7, 1), (2, 8, 1), (2, 8, 1), (-3, 3, 1), (-3, 3, 1)], 'times': [0, 3.762481218602597, 4.5470717674033585, 5.389346441902969, 6.17393699070373, 7.505690184151588, 8.290280732952349, 10.173664160859989, 10.95825470966075, 11.800529384160361, 12.585119932961122, 17.45067966788328, 22.36538634073566], 'energy': [0, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.07779019230998471, 0]}, 
                'box_bot6': {'locations': [(-3, 4, 1), (0, 1, 1), (0, 1, 1), (2, 2, 1), (2, 2, 1), (0, 6, 1), (0, 6, 1), (0, 7, 1), (0, 7, 1), (0, 9, 1), (0, 9, 1), (1, 9, 1), (1, 9, 1), (-3, 4, 1), (-3, 4, 1), (9, 9, 1), (9, 9, 1), (-3, 4, 1), (-3, 4, 1)], 'times': [0, 2.297731831360669, 3.219677434121513, 4.595463662721338, 5.517409265482182, 8.268981722681833, 9.190927325442678, 9.806197631374133, 10.728143234134977, 11.958683845997886, 12.88062944875873, 13.495899754690186, 14.41784535745103, 19.770450346847568, 20.891219625239774, 30.514401339737688, 31.43634694249853, 39.215637451474755, 40.35391716852351], 'energy': [0, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.033689386338045246, 0, 0.0, 0.0031002966503563757, 0.04671877303105314, 0]}, 
                'box_bot7': {'locations': [(-3, 5, 1), (9, 0, 1), (9, 0, 1), (9, 4, 1), (9, 4, 1), (-3, 5, 1), (-3, 5, 1)], 'times': [0, 9.82752925040731, 10.761859600742422, 14.301485058944984, 15.235815409280097, 24.195038983232195, 29.881849532078917], 'energy': [0, 0.0, 0.01699852089232154, 0.0, 0.01699852089232154, 0.09828951111463693, 0]}, 
                'box_bot8': {'locations': [(-3, 6, 1)], 'times': [0], 'energy': [0]}}
        Times = {'box_bot0': {'locations': [(-3, -2, 1), (4, 0, 1), (4, 0, 1), (7, 3, 1), (7, 3, 1), (9, 4, 1), (9, 4, 1), (9, 8, 1), (9, 8, 1), (-3, -2, 1), (-3, -2, 1)], 'times': [0, 4.851426971299862, 5.556010497849312, 9.006392806626849, 9.7109763331763, 11.529487482190866, 12.234071008740317, 15.487122646370574, 16.191706172920025, 26.428456080969312, 28.160547914318443], 'energy': [0, 0.0, 0.008928659092230019, 0.0, 0.008928659092230019, 0.0, 0.008928659092230019, 0.0, 0.008928659092230019, 0.056520399382174014, 0]}, 
                 'box_bot1': {'locations': [(-3, -1, 1), (0, 1, 1), (0, 1, 1), (1, 3, 1), (1, 3, 1), (0, 6, 1), (0, 6, 1), (0, 7, 1), (0, 7, 1), (5, 7, 1), (5, 7, 1), (6, 8, 1), (6, 8, 1), (7, 9, 1), (7, 9, 1), (-3, -1, 1), (-3, -1, 1)], 'times': [0, 1.9838376234434292, 2.7806213171428245, 3.9676752468868584, 4.764458940586254, 6.443206707298546, 7.239990400997941, 7.770857056971125, 8.56764075067052, 11.22197403053644, 12.018757724235835, 12.769516548924765, 13.566300242624159, 14.317059067313089, 15.113842761012483, 21.11546937072673, 32.495379380013205], 'energy': [0, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.0, 0.015811557303170548, 0.13051368598287016, 0]}, 
                 'box_bot2': {'locations': [(-3, 0, 1), (0, 0, 1), (0, 0, 1), (2, 2, 1), (2, 2, 1), (5, 2, 1), (5, 2, 1), (7, 1, 1), (7, 1, 1), (9, 0, 1), (9, 0, 1), (-3, 0, 1), (-3, 0, 1), (9, 9, 1), (9, 9, 1), (-3, 0, 1), (-3, 0, 1)], 'times': [0, 1.4336142162838172, 1.9972051927078385, 3.7372516724274307, 4.300842648851452, 6.146440646935708, 6.710031623359729, 8.085659150977703, 8.649250127401725, 10.024877655019699, 10.588468631443721, 16.207554430798524, 17.425160117846872, 26.688983492868854, 27.252574469292874, 35.38921589146681, 36.17664982592169], 'energy': [0, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.0, 0.00892578947373684, 0.014100639879395718, 0, 0.0, 0.00892578947373684, 0.01984235119653773, 0]}, 
                 'box_bot3': {'locations': [(-3, 1, 1), (1, 4, 1), (1, 4, 1), (1, 7, 1), (1, 7, 1), (2, 8, 1), (2, 8, 1), (-3, 1, 1), (-3, 1, 1)], 'times': [0, 5.202993709711098, 5.81258647247825, 8.371505899154888, 8.98109866192204, 10.187384847997402, 10.796977610764554, 18.279398585147817, 18.969879194997173], 'energy': [0, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.0, 0.011354998398906564, 0.005655488546019529, 0]}, 
                 'box_bot4': {'locations': [(-3, 2, 1), (3, 0, 1), (3, 0, 1), (5, 1, 1), (5, 1, 1), (6, 6, 1), (6, 6, 1), (6, 7, 1), (6, 7, 1), (8, 9, 1), (8, 9, 1), (-3, 2, 1), (-3, 2, 1)], 'times': [0, 3.819681690061162, 4.526521246351695, 6.214696919341858, 6.921536475632391, 10.771169306819555, 11.478008863110087, 12.23298397566358, 12.939823531954112, 15.075215618808716, 15.782055175099249, 25.232366948000557, 29.16696803345971], 'energy': [0, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.0, 0.005646663165172199, 0.11096787293859779, 0]}, 
                 'box_bot5': {'locations': [(-3, 3, 1), (1, 0, 1), (1, 0, 1), (3, 2, 1), (3, 2, 1), (4, 4, 1), (4, 4, 1), (5, 6, 1), (5, 6, 1), (4, 7, 1), (4, 7, 1), (5, 8, 1), (5, 8, 1), (5, 9, 1), (5, 9, 1), (-3, 3, 1), (-3, 3, 1)], 'times': [0, 2.116343742248618, 2.900934291049379, 4.5854836400486025, 5.3700741888493635, 6.701827382297221, 7.486417931097982, 8.81817112454584, 9.6027616733466, 10.445036347846212, 11.229626896646973, 12.071901571146585, 12.856492119947346, 13.452070253907713, 14.236660802708474, 20.39764515139698, 26.14044938014972], 'energy': [0, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.0, 0.010723183754096007, 0.09562535151243566, 0]}, 
                 'box_bot6': {'locations': [(-3, 4, 1), (0, 3, 1), (0, 3, 1), (1, 5, 1), (1, 5, 1), (1, 6, 1), (1, 6, 1), (2, 7, 1), (2, 7, 1), (0, 9, 1), (0, 9, 1), (-3, 4, 1), (-3, 4, 1)], 'times': [0, 3.4587700624223245, 4.3807156651831685, 5.7565018937829935, 6.678447496543837, 7.2937178024752924, 8.215663405236137, 9.085787016409844, 10.007732619170689, 11.747979841518102, 12.669925444278947, 17.93136988949822, 18.957521092353826], 'energy': [0, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.0, 0.0031002966503563757, 0.033199926137435366, 0]}, 
                 'box_bot7': {'locations': [(-3, 5, 1), (2, 6, 1), (2, 6, 1), (1, 9, 1), (1, 9, 1), (-3, 5, 1), (-3, 5, 1)], 'times': [0, 7.673576442687684, 8.607906793022796, 11.406226420982103, 12.340556771317214, 20.430536062074435, 25.285916623954364], 'energy': [0, 0.0, 0.01699852089232154, 0.0, 0.01699852089232154, 0.08965382809433407, 0]}, 
                 'box_bot8': {'locations': [(-3, 6, 1), (2, 3, 1), (2, 3, 1), (3, 6, 1), (3, 6, 1), (3, 7, 1), (3, 7, 1), (3, 9, 1), (3, 9, 1), (-3, 6, 1), (-3, 6, 1)], 'times': [0, 4.745371211326204, 5.297621877159367, 7.949584162640506, 8.501834828473669, 9.340458937572278, 9.892709603405441, 11.569957821602657, 12.12220848743582, 20.60221589906736, 31.96253553906209], 'energy': [0, 0.0, 0.004325501653926118, 0.0, 0.004325501653926118, 0.0, 0.004325501653926118, 0.0, 0.004325501653926118, 0.18045146944050525, 0]}}

        self.velocities =Velocity
        self.waypoints = Ours #Energy, Ours, Times

        self.robot_colors = {
            'box_bot0': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Red
            'box_bot1': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Green
            'box_bot2': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Blue
            'box_bot3': ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # Black
            'box_bot4': ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),  # Magenta
            'box_bot5': ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),  # Cyan
            'box_bot6': ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0),  # Grey
            'box_bot7': ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0),  # Purple
            'box_bot8': ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0)   # Teal
        }
        self.mesh_files = {'box_bot0':"package://box_bot_description/meshes/robot0.dae",
                           'box_bot1':"package://box_bot_description/meshes/robot1.dae",
                           'box_bot2':"package://box_bot_description/meshes/robot2.dae",
                           'box_bot3':"package://box_bot_description/meshes/robot3.dae",
                           'box_bot4':"package://box_bot_description/meshes/robot4.dae",
                           'box_bot5':"package://box_bot_description/meshes/robot5.dae",
                           'box_bot6':"package://box_bot_description/meshes/robot6.dae",
                           'box_bot7':"package://box_bot_description/meshes/robot7.dae",
                           'box_bot8':"package://box_bot_description/meshes/robot8.dae"}
        
        self.initial_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow for unvisited waypoints


        self.visited_waypoints = set()

        self.subscribers_ = []
        self.publishers_ = {}
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        current_seconds, current_nanoseconds = self.get_clock().now().seconds_nanoseconds()
        self.mission_start_time = current_seconds + current_nanoseconds / 1e9 
        self.current_idx = {f'box_bot{i}': 0 for i in range(9)}
        for i in range(9):
            robot_name = f'box_bot{i}'
            self.current_robot = robot_name
            self.publishers_[robot_name] = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
            self.subscribers_.append(
                self.create_subscription(TFMessage, f'/{robot_name}/tf', lambda msg, name=robot_name: self.tf_callback(name, msg), 10))
            self.get_logger().info(f'Set up subscriber and publisher for {robot_name}')
        
        self.publish_initial_markers()
    
    def publish_initial_markers(self):
        marker_array = MarkerArray()
        for robot, waypoint_data in self.waypoints.items():
            for i, (x, y, z) in enumerate(waypoint_data['locations']):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = robot
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(x)-4.5
                marker.pose.position.y = float(y)-4.5
                marker.pose.position.z = float(z)
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                marker.color = self.initial_color
                marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
                marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def tf_callback(self, robot_name, msg):
        wrapped_message = RobotTFWrapper(robot_name=robot_name, tf_message=msg)
        self.process_wrapped_message(wrapped_message)

    def process_wrapped_message(self, wrapped_message):
        self.get_logger().info(f'TF data received for {wrapped_message.robot_name}')
        for transform in wrapped_message.tf_message.transforms:
            if wrapped_message.robot_name.startswith('box_bot'):
                self.control_robot(wrapped_message,transform, wrapped_message.robot_name)

    def control_robot(self, wrapped_message, transform, robot_name):
        waypoint = self.waypoints.get(robot_name)
        if not waypoint:
            return

        locations = waypoint['locations']
        vel_lin = self.velocities[robot_name]
        times = waypoint['times']

        current_x = float(transform.transform.translation.x)
        current_y = float(transform.transform.translation.y)
        current_z = float(transform.transform.translation.z)
        current_theta = math.atan2(2.0 * (transform.transform.rotation.w * transform.transform.rotation.z),
                                   1.0 - 2.0 * (transform.transform.rotation.z**2))

        current_seconds, current_nanoseconds = self.get_clock().now().seconds_nanoseconds()
        current_time = (current_seconds + current_nanoseconds / 1e9 - self.mission_start_time)

        target_index = self.current_idx[robot_name]
        target_location = locations[target_index]
        target_x, target_y, _ = target_location
        self.get_logger().info(f'Send to {robot_name}: target: {target_index}')
        if self.reach_target(robot_name,locations,[current_x,current_y],target_index):
            self.current_idx[robot_name]+=1
            self.update_marker_color(robot_name, target_index)

        travel_time = times[target_index] if target_index != 0 else 0.0
        
        dx = target_x - current_x-4.5
        dy = target_y - current_y-4.5
        distance = math.sqrt(dx**2 + dy**2)
        self.get_logger().info(f'Send to {robot_name}: differences: {dx},{dy}, distance: {distance}')

        linear_velocity = self.velocities[robot_name]/4 if travel_time > 0.0 else 0.0
        
        target_theta = math.atan2(dy, dx)

        angle_difference = target_theta - current_theta
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))
        self.get_logger().info(f'Time: {travel_time}, Angle: {angle_difference}, current_time: {current_time}')
        
        if abs(angle_difference)<0.1:
            angular_velocity = 0.0
            linear_velocity = self.velocities[robot_name]/4 if travel_time > 0.0 else 0.0
        else:
            linear_velocity = 0.0
            angular_velocity = angle_difference/2.0  if travel_time > 0.0 else 0.0
        
        collision_detected, other_robot = self.detect_collision(wrapped_message, robot_name, current_x, current_y)
        if collision_detected:
            self.get_logger().warn(f'Collision risk detected for {robot_name}, stopping movement.')
            angular_velocity = 0.0
            linear_velocity = 0.0
        elif abs(angle_difference) < 0.1:
            angular_velocity = 0.0
        else:
            angular_velocity = angle_difference/2.0 if travel_time > 0.0 else 0.0
            linear_velocity = 0.0

        self.get_logger().info(f'Send to {robot_name}: rotation: {angular_velocity}, linear: {linear_velocity}')
        
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publishers_[robot_name].publish(twist)
        current_seconds, current_nanoseconds = self.get_clock().now().seconds_nanoseconds()
        self.mission_start_time = current_seconds + current_nanoseconds / 1e9
    def reach_target(self, robot_name, X, Y, tgt_idx):
        target_x, target_y, _ = X[tgt_idx]  # Get the target location
        current_x, current_y = Y  # Current robot position
        dx = target_x - (current_x + 4.5)  # Apply offset correction only to the target
        dy = target_y - (current_y + 4.5)
        distance = math.sqrt(dx**2 + dy**2)
        if tgt_idx < (len(X)-1) and distance <0.1:
            return True
        return False

    def detect_collision(self, wrapped_message, robot_name, current_x, current_y):
        for other_robot in self.waypoints.keys():
            if other_robot != robot_name:
                other_x, other_y, _ = self.get_current_position(wrapped_message,other_robot)
                self.get_logger().info(f"Checking collision for {robot_name}: current_x={current_x}, current_y={current_y}, other_robot={other_robot}, other_x={other_x}, other_y={other_y}")
                distance = math.sqrt((current_x - other_x)**2+(current_y - other_y)**2)
                if (abs(current_x) < 0.1 and abs(current_y) < 0.1) or (abs(other_x) < 0.1 and abs(other_y) < 0.1):
                    self.get_logger().info(f"[Collision Detection] Skipping collision check at (0, 0, 1) position.")
                    continue
                if distance < 0.5:
                    return True, other_robot
        return False, None
        
    def update_marker_color(self, robot_name, index):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = robot_name
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY
        loc = self.waypoints[robot_name]['locations'][index]
        marker.pose.position.x = float(loc[0])-4.5
        marker.pose.position.y = float(loc[1])-4.5
        marker.pose.position.z = float(loc[1])+0.5
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color = self.robot_colors[robot_name]
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker_array.markers.append(marker)

        image_marker = Marker()
        image_marker.header.frame_id = robot_name
        image_marker.ns = robot_name
        image_marker.id = index + 100  # Different ID to avoid conflicts
        image_marker.type = Marker.MESH_RESOURCE
        image_marker.action = Marker.ADD
        image_marker.pose.position.x = float(loc[0]) - 4.5
        image_marker.pose.position.y = float(loc[1]) - 4.5
        image_marker.pose.position.z = float(loc[2]) + 1.0  # Adjust height
        image_marker.pose.orientation.w = 1.0
        image_marker.scale.x = 1.0
        image_marker.scale.y = 1.0
        image_marker.scale.z = 1.0
        image_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        image_marker.mesh_resource = self.mesh_files[robot_name]
        image_marker.mesh_use_embedded_materials = True
        marker_array.markers.append(image_marker)
        self.marker_pub.publish(marker_array)

    def get_current_position(self, wrapped_message, robot_name):
        for transform in wrapped_message.tf_message.transforms:
            if wrapped_message.robot_name.startswith(robot_name):
                current_x_ = float(transform.transform.translation.x)
                current_y_ = float(transform.transform.translation.y)
                current_z_ = float(transform.transform.translation.z)
                return (current_x_, current_y_, current_z_)
        return (0.0,0.0,0.0)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
