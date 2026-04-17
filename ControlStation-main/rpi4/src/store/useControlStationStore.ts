import { create } from "zustand";

export type StationTab =
  | "main"
  | "crane"
  | "paws"
  | "servo"
  | "debug"
  | "settings";

export type ServoDirection = "counterclockwise" | "clockwise";

export type ServoValueField = "targetPosition" | "step" | "quickPosition";

export interface PawState {
  id: 1 | 2 | 3;
  name: string;
  photoLabel: string;
  captured: boolean;
  extended: boolean;
}

export interface ServoState {
  id: string;
  name: string;
  direction: ServoDirection;
  currentPosition: number;
  targetPosition: number;
  step: number;
  zeroPosition: number;
  quickPosition: number;
  photoLabel: string;
}

export interface MapRobotState {
  x: number;
  y: number;
  heading: number;
}

export const roundToTenth = (value: number) => Math.round(value * 10) / 10;

const clampCranePosition = (value: number) => Math.max(1, Math.min(10, Math.round(value)));

const sanitizeServoValue = (field: ServoValueField, value: number) => {
  const rounded = roundToTenth(value);

  if (field === "step") {
    return Math.max(0.1, Math.abs(rounded));
  }

  return rounded;
};

const mockPaws: PawState[] = [
  {
    id: 1,
    name: "Лапа № 1",
    photoLabel: "Фотография текущего состояния лапы № 1",
    captured: false,
    extended: true,
  },
  {
    id: 2,
    name: "Лапа № 2",
    photoLabel: "Фотография текущего состояния лапы № 2",
    captured: true,
    extended: false,
  },
  {
    id: 3,
    name: "Лапа № 3",
    photoLabel: "Фотография текущего состояния лапы № 3",
    captured: false,
    extended: false,
  },
];

const mockServos: ServoState[] = [
  {
    id: "servo-1",
    name: "Сервопривод № 1",
    direction: "clockwise",
    currentPosition: 125,
    targetPosition: 0,
    step: 160,
    zeroPosition: 0,
    quickPosition: 160,
    photoLabel: "Фотография сервопривода № 1",
  },
  {
    id: "servo-2",
    name: "Сервопривод № 2",
    direction: "counterclockwise",
    currentPosition: 42.5,
    targetPosition: 0,
    step: 12.5,
    zeroPosition: 0,
    quickPosition: 24.8,
    photoLabel: "Фотография сервопривода № 2",
  },
  {
    id: "servo-3",
    name: "Сервопривод № 3",
    direction: "clockwise",
    currentPosition: 78.2,
    targetPosition: 0,
    step: 5,
    zeroPosition: 0,
    quickPosition: 90,
    photoLabel: "Фотография сервопривода № 3",
  },
  {
    id: "servo-4",
    name: "Сервопривод № 4",
    direction: "counterclockwise",
    currentPosition: 15,
    targetPosition: 0,
    step: 1.5,
    zeroPosition: 0,
    quickPosition: 20,
    photoLabel: "Фотография сервопривода № 4",
  },
  {
    id: "servo-5",
    name: "Сервопривод № 5",
    direction: "clockwise",
    currentPosition: 31.7,
    targetPosition: 0,
    step: 2.5,
    zeroPosition: 0,
    quickPosition: 50,
    photoLabel: "Фотография сервопривода № 5",
  },
  {
    id: "servo-6",
    name: "Сервопривод № 6",
    direction: "counterclockwise",
    currentPosition: 67.4,
    targetPosition: 0,
    step: 3.4,
    zeroPosition: 0,
    quickPosition: 70,
    photoLabel: "Фотография сервопривода № 6",
  },
];

const mockMapRobot: MapRobotState = {
  x: 43.5,
  y: 58.2,
  heading: 32,
};

const updateSelectedServo = (
  servos: ServoState[],
  selectedServoId: string,
  updater: (servo: ServoState) => ServoState,
) =>
  servos.map((servo) => {
    if (servo.id !== selectedServoId) {
      return servo;
    }

    return updater(servo);
  });

interface ControlStationState {
  activeTab: StationTab;
  mapPhotoLabel: string;
  mapRobot: MapRobotState;
  stickerPhotoLabel: string;
  cranePhotoLabel: string;
  cranePosition: number;
  craneExtendedPosition: number;
  cranePayloadState: "ready" | "released";
  paws: PawState[];
  selectedPawId: PawState["id"];
  servos: ServoState[];
  selectedServoId: string;
  isServoMenuOpen: boolean;
  setActiveTab: (tab: StationTab) => void;
  setCranePosition: (position: number) => void;
  extendCrane: () => void;
  retractCrane: () => void;
  releaseCranePayload: () => void;
  selectPaw: (id: PawState["id"]) => void;
  setPawCaptured: (captured: boolean) => void;
  setPawExtended: (extended: boolean) => void;
  toggleServoMenu: () => void;
  closeServoMenu: () => void;
  selectServo: (id: string) => void;
  setServoDirection: (direction: ServoDirection) => void;
  setServoValue: (field: ServoValueField, value: number) => void;
  moveServoByStep: (direction: "backward" | "forward") => void;
  applyServoTarget: (field: "targetPosition" | "quickPosition") => void;
  setServoZeroFromCurrent: () => void;
}

export const useControlStationStore = create<ControlStationState>((set) => ({
  activeTab: "main",
  mapRobot: mockMapRobot,
  mapPhotoLabel: "Фотография карты",
  stickerPhotoLabel: "Фотография стикера",
  cranePhotoLabel: "Фотография крана",
  cranePosition: 5,
  craneExtendedPosition: 5,
  cranePayloadState: "ready",
  paws: mockPaws,
  selectedPawId: 1,
  servos: mockServos,
  selectedServoId: "servo-1",
  isServoMenuOpen: false,
  setActiveTab: (tab) => set({ activeTab: tab, isServoMenuOpen: false }),
  setCranePosition: (position) => {
    const nextPosition = clampCranePosition(position);

    set({
      cranePosition: nextPosition,
      craneExtendedPosition: nextPosition,
    });
  },
  extendCrane: () =>
    set((state) => {
      const nextPosition = clampCranePosition(state.craneExtendedPosition + 1);

      return {
        cranePosition: nextPosition,
        craneExtendedPosition: nextPosition,
      };
    }),
  retractCrane: () => set({ cranePosition: 1 }),
  releaseCranePayload: () => set({ cranePayloadState: "released" }),
  selectPaw: (id) => set({ selectedPawId: id }),
  setPawCaptured: (captured) =>
    set((state) => ({
      paws: state.paws.map((paw) =>
        paw.id === state.selectedPawId
          ? {
              ...paw,
              captured,
            }
          : paw,
      ),
    })),
  setPawExtended: (extended) =>
    set((state) => ({
      paws: state.paws.map((paw) =>
        paw.id === state.selectedPawId
          ? {
              ...paw,
              extended,
            }
          : paw,
      ),
    })),
  toggleServoMenu: () => set((state) => ({ isServoMenuOpen: !state.isServoMenuOpen })),
  closeServoMenu: () => set({ isServoMenuOpen: false }),
  selectServo: (id) =>
    set({
      selectedServoId: id,
      isServoMenuOpen: false,
    }),
  setServoDirection: (direction) =>
    set((state) => ({
      servos: updateSelectedServo(state.servos, state.selectedServoId, (servo) => ({
        ...servo,
        direction,
      })),
    })),
  setServoValue: (field, value) =>
    set((state) => ({
      servos: updateSelectedServo(state.servos, state.selectedServoId, (servo) => ({
        ...servo,
        [field]: sanitizeServoValue(field, value),
      })),
    })),
  moveServoByStep: (direction) =>
    set((state) => ({
      servos: updateSelectedServo(state.servos, state.selectedServoId, (servo) => ({
        ...servo,
        currentPosition: roundToTenth(
          servo.currentPosition + (direction === "forward" ? servo.step : -servo.step),
        ),
      })),
    })),
  applyServoTarget: (field) =>
    set((state) => ({
      servos: updateSelectedServo(state.servos, state.selectedServoId, (servo) => ({
        ...servo,
        currentPosition: field === "targetPosition" ? 0 : servo[field],
      })),
    })),
  setServoZeroFromCurrent: () =>
    set((state) => ({
      servos: updateSelectedServo(state.servos, state.selectedServoId, (servo) => ({
        ...servo,
        zeroPosition: servo.currentPosition,
        currentPosition: 0,
        targetPosition: 0,
      })),
    })),
}));
