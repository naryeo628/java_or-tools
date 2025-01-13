package org.or_tools.example;
import com.google.ortools.Loader;
import com.google.ortools.constraintsolver.*;
import com.google.protobuf.Duration;

import java.util.stream.IntStream;

public final class VRPTotal {
//    private static final Logger logger = Logger.getLogger(VRPWithConstraints.class.getName());

    static class DataModel {
        public final long[][] distanceMatrix = { //(1+16)C2 = 136번 경로 호출해야함
            {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468, 776, 662},
            {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674, 1016, 868, 1210},
            {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130, 788, 1552, 754},
            {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822, 1164, 560, 1358},
            {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708, 1050, 674, 1244},
            {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514, 1050, 708},
            {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514, 1278, 480},
            {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662, 742, 856},
            {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320, 1084, 514},
            {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274, 810, 468},
            {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730, 388, 1152, 354},
            {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308, 650, 274, 844},
            {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536, 388, 730},
            {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342, 422, 536},
            {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342, 0, 764, 194},
            {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388, 422, 764, 0, 798},
            {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536, 194, 798, 0},
        };

        public final long[][] timeMatrix = {
            {0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7},
            {6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14},
            {9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9},
            {8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16},
            {7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14},
            {3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8},
            {6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5},
            {2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10},
            {3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6},
            {2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5},
            {6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4},
            {6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10},
            {4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8},
            {4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6},
            {5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2},
            {9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9},
            {7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0},
        };

        public final long[][] timeWindows = {
            {0, 0},  // Depot
            {9, 11}, // 1: +9 ~ +11 시간 후 도착해야한다.
            {10, 15},// 2: +10 ~ +15 시간 후 도착해야한다.
            {12, 18},// 3: +12 ~ +18 시간 후 도착해야한다.
            {10, 13}, // 4
            {0, 5}, // 5
            {5, 10}, // 6
            {0, 4}, // 7
            {5, 10}, // 8
            {0, 3}, // 9
            {10, 16}, // 10
            {10, 15}, // 11
            {0, 5}, // 12
            {5, 10}, // 13
            {7, 8}, // 14
            {10, 15}, // 15
            {11, 15}, // 16
        };

        public final long[] weights = {
            0,
            1,
            1,
            2,
            4,
            2,
            4,
            8,
            8,
            1,
            2,
            1,
            2,
            4,
            4,
            8,
            8,
        }; // 각 고객의 무게
        public final int[] cbms = {
            0,
            2,
            3,
            1,
            2,
            3,
            1,
            3,
            2,
            3,
            1,
            2,
            3,
            1,
            2,
            3,
            1,
        }; // 각 고객의 부피 (CBM)

        public final long[] vehicleWeightCapacities = {
            15,
            15,
            15,
            15,
            15,
            15,
            15,
        }; // 차량 무게 용량
        public final long[] vehicleCbmCapacities = {
            10,
            10,
            10,
            10,
            10,
            10,
            10,
        }; // 차량 부피 용량
        public final int vehicleNumber = vehicleWeightCapacities.length; // 차량 수
        public final int depot = 0; // 차고지 인덱스
        public final int[] depots = {
            0,
            0,
            0,
            0,
            0,
            0,
        }; // 출발지 인덱스
        public final int[] endDepots = {
            0,
            0,
            0,
            0,
            0,
            0,
        }; // 도착지 인덱스

        public final int searchTimeLimit = 60; // 최대 탐색 시간 (초)
    }

    public static void main(String[] args) {
        Loader.loadNativeLibraries();
        // 데이터 모델 생성
        final DataModel data = new DataModel();

        // RoutingIndexManager 생성
        RoutingIndexManager manager = new RoutingIndexManager(
            data.distanceMatrix.length,
            data.vehicleNumber,
            data.depot /* Start Location */
//            data.depots, /* Start Locations */
//            IntStream.range(0, data.vehicleNumber).toArray() /* End locations */
        );

        // RoutingModel 생성
        RoutingModel routing = new RoutingModel(manager);

        // 거리 비용 함수 정의
        int transitCallbackIndex = routing.registerTransitCallback((long fromIndex, long toIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            int toNode = manager.indexToNode(toIndex);
            return data.distanceMatrix[fromNode][toNode];
        });
        routing.setArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

        // 무게 제약 추가
        int demandCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            return data.weights[fromNode];
        });
        routing.addDimensionWithVehicleCapacity(
            demandCallbackIndex,
            0, // 무게 여유량
            data.vehicleWeightCapacities,
            true, // 누적 시작
            "Capacity"
        );

        // 부피(CBM) 제약 추가
        int cbmCallbackIndex = routing.registerUnaryTransitCallback((long fromIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            return data.cbms[fromNode];
        });
        routing.addDimensionWithVehicleCapacity(
            cbmCallbackIndex,
            0, // 부피 여유량
            data.vehicleCbmCapacities,
            true, // 누적 시작
            "CBM"
        );

        // 시간 제약 추가
        int timeCallbackIndex = routing.registerTransitCallback((long fromIndex, long toIndex) -> {
            int fromNode = manager.indexToNode(fromIndex);
            int toNode = manager.indexToNode(toIndex);
            return data.timeMatrix[fromNode][toNode];
        });
        routing.addDimension(
            timeCallbackIndex,
            30, // 대기 허용 시간
            300, // 최대 운행 시간
            false, // 시작 시간 설정 여부
            "Time"
        );
        RoutingDimension timeDimension = routing.getMutableDimension("Time");

        // 각 위치의 시간 창 설정
        IntStream.range(0, data.timeWindows.length).forEach(locationIdx -> {
            long index = manager.nodeToIndex(locationIdx);
            timeDimension.cumulVar(index).setRange(data.timeWindows[locationIdx][0], data.timeWindows[locationIdx][1]);
        });

        // 거리 제약 추가
        routing.addDimension(
            transitCallbackIndex,
            0, // 여유 거리
            5000, // 차량 최대 거리 제한
            true, // 누적 시작
            "Distance"
        );

        // 최적화 파라미터 설정
        RoutingSearchParameters searchParameters =
            main.defaultRoutingSearchParameters()
                .toBuilder()
                .setFirstSolutionStrategy(FirstSolutionStrategy.Value.PATH_CHEAPEST_ARC)
                .setLocalSearchMetaheuristic(LocalSearchMetaheuristic.Value.GUIDED_LOCAL_SEARCH)
                .setTimeLimit(Duration.newBuilder().setSeconds(data.searchTimeLimit).build()) // 60초 제한
//                .setLogSearch(true)
                .build();

        // 문제 해결
        Assignment solution = routing.solveWithParameters(searchParameters);

        // 결과 출력
        if (solution != null) {
            System.out.println("\n====================================");
            for (int i = 0; i < data.vehicleNumber; i++) {
                System.out.println("\nRoute for vehicle " + i + ":");
                long index = routing.start(i);
                while (!routing.isEnd(index)) {
                    System.out.print(manager.indexToNode(index) + " -> ");
                    index = solution.value(routing.nextVar(index));
                }
                System.out.println(manager.indexToNode(index));
                System.out.println("총이동거리: " + solution.value(routing.getDimensionOrDie("Distance").cumulVar(index)));
                System.out.println("총물품무게: " + solution.value(routing.getDimensionOrDie("Capacity").cumulVar(index)));
                System.out.println("총물품부피: " + solution.value(routing.getDimensionOrDie("CBM").cumulVar(index)));
                System.out.println("총운행시간: " + solution.value(routing.getDimensionOrDie("Time").cumulVar(index)));
                System.out.println("\n====================================");
            }
        } else {
            System.out.println("No solution found.");
        }
    }
}
