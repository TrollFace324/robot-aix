import { useEffect, useRef, useState } from "react";

import LiquidChrome from "./LiquidChrome";
import {
  shouldResetKranActionToHomeOnFailure,
  shouldResetKranActionToHomeOnSuccess,
} from "./kranActionState";
import {
  appendDigitToTelemetryMmDraft,
  backspaceTelemetryMmDraft,
  buildTelemetryBoomSlotOptions,
  isTelemetryBoomMmInRange,
  normalizeTelemetryMmDraft,
  parseTelemetryBoomSlots,
  shouldClearTelemetryBoomSlots,
  shouldLoadTelemetryBoomSlots,
  telemetryBoomMaxMm,
  telemetryBoomMinMm,
} from "./telemetryBoomEditor";

import batteryIcon from "../images/green-lightning.svg";
import activePositionImage from "../images/active-position.svg";
import dashboardIcon from "../images/icon-dashboard.svg";
import mapImage from "../images/map.png";
import startButtonImage from "../images/button-start-3.png";
import backgroundImage from "../images/background.webp";
import wheelsImage from "../images/wheels.png";

type TabId = "main" | "kran" | "lapa" | "servo" | "telemetry" | "label";
type RequestState = "idle" | "pending" | "success" | "error";
type KranMode = "auto" | "manual";
type KranAction = "home" | "throw";
type TelemetryOptionKey = "general" | "wheelbase" | "lift" | "boom";
type SystemStatusLevel = 0 | 1 | 2;
type SystemSummary = {
  robot: SystemStatusLevel;
  pre: SystemStatusLevel;
  crsf: SystemStatusLevel;
  batt: SystemStatusLevel;
  voltage_cV: number;
};
type SendSlaveRequestOptions = {
  isAutoRequest?: boolean;
  isKranAction?: boolean;
  isSilent?: boolean;
};
type SendSlaveRequestResult = {
  ok: boolean;
  payload: RequestPayload | null;
  errorMessage: string;
  slaveResponse: string;
  slaveResponses: string[];
};
type TelemetryOption = {
  key: TelemetryOptionKey;
  label: string;
  requestId?: string;
  metricLabel?: string;
  metricValue?: string;
  footerValue?: string;
  mode: "default" | "boomEditor";
};

const requestIds = {
  tabMain: "0",
  tabKran: "1",
  tabLapa: "2",
  tabServo: "3",
  tabTelemetry: "4",
  tabLabel: "5",
  startAutonomous: "6",
  kranModeAuto: "7",
  kranModeManual: "8",
  kranPositionUp: "9",
  kranPositionDown: "10",
  kranHome: "11",
  kranThrow: "12",
  paw1: "13",
  paw2: "14",
  paw3: "15",
  pawCapture: "16",
  pawRelease: "17",
  servoDropdownToggle: "18",
  servoSelect: "19",
  servoMinus: "20",
  servoPlus: "21",
  servoReturnZero: "22",
  servoSetZero: "23",
  servoSetEnd: "24",
  servoSend: "25",
  telemetryDropdownToggle: "26",
  telemetryGeneral: "27",
  telemetryWheelbase: "28",
  telemetryLift: "29",
  telemetryBoomRead: "30",
  telemetryBoomWrite: "31",
} as const;

const tabs: ReadonlyArray<{ id: TabId; label: string; requestId: string }> = [
  { id: "main", label: "Main", requestId: requestIds.tabMain },
  { id: "kran", label: "Kran", requestId: requestIds.tabKran },
  { id: "lapa", label: "Lapa", requestId: requestIds.tabLapa },
  { id: "servo", label: "Servo", requestId: requestIds.tabServo },
  { id: "telemetry", label: "Telemetry", requestId: requestIds.tabTelemetry },
  { id: "label", label: "Label", requestId: requestIds.tabLabel },
];

const systemStatuses: ReadonlyArray<{
  key: keyof Omit<SystemSummary, "voltage_cV">;
  label: string;
}> = [
  { key: "robot", label: "Подключение с роботом" },
  { key: "pre", label: "Предварительная проверка" },
  { key: "crsf", label: "Подключение с контроллером" },
  { key: "batt", label: "Состояние батареи" },
] as const;

const servoOptions = [
  {
    servoNumber: 1,
    label: "Сервопривод №1",
    initialValue: "0°",
    finalValue: "0°",
    currentValue: "0°",
  },
  {
    servoNumber: 2,
    label: "Сервопривод №2",
    initialValue: "0°",
    finalValue: "0°",
    currentValue: "0°",
  },
  {
    servoNumber: 3,
    label: "Сервопривод №3",
    initialValue: "0°",
    finalValue: "0°",
    currentValue: "0°",
  },
  {
    servoNumber: 4,
    label: "Сервопривод №4",
    initialValue: "0°",
    finalValue: "0°",
    currentValue: "0°",
  },
  {
    servoNumber: 5,
    label: "Сервопривод №5",
    initialValue: "0°",
    finalValue: "0°",
    currentValue: "0°",
  },
  {
    servoNumber: 6,
    label: "Сервопривод №6",
    initialValue: "0°",
    finalValue: "0°",
    currentValue: "0°",
  },
] as const;

const telemetryOptions: ReadonlyArray<TelemetryOption> = [
  {
    key: "general",
    label: "Общая информация",
    requestId: requestIds.telemetryGeneral,
    metricLabel: "Потребляемый ток узла",
    metricValue: "0 ma",
    footerValue: "0°",
    mode: "default",
  },
  {
    key: "wheelbase",
    label: "Колесная база",
    requestId: requestIds.telemetryWheelbase,
    metricLabel: "Потребляемый ток узла",
    metricValue: "0 ma",
    footerValue: "0°",
    mode: "default",
  },
  {
    key: "lift",
    label: "Подъемник",
    requestId: requestIds.telemetryLift,
    metricLabel: "Потребляемый ток узла",
    metricValue: "0 ma",
    footerValue: "0°",
    mode: "default",
  },
  {
    key: "boom",
    label: "Стрела",
    mode: "boomEditor",
  },
];

const telemetryBoomSlotOptions = buildTelemetryBoomSlotOptions();
const telemetryBoomKeypadDigits = [
  ["1", "2", "3"],
  ["4", "5", "6"],
  ["7", "8", "9"],
  ["C", "0", "⌫"],
] as const;

const liquidChromeBaseColor: [number, number, number] = [0.78, 0.7, 0.92];
const batteryRadius = 46;
const lapaGuideTravel = 272;
const systemSummaryPollMs = 10_000;
const systemSummaryReconnectPollMs = 1_000;
const batteryEmptyCv = 1050;
const batteryFullCv = 1260;
const kranActionRequestStartWaitMs = 750;
const kranActionRequestRetryMs = 25;
const defaultSystemSummary: SystemSummary = {
  robot: 1,
  pre: 1,
  crsf: 1,
  batt: 1,
  voltage_cV: 0,
};

const resolveMasterApiBaseUrl = () => {
  const configuredBaseUrl = import.meta.env.VITE_MASTER_API_BASE_URL?.trim();

  if (configuredBaseUrl) {
    return configuredBaseUrl.replace(/\/$/, "");
  }

  const protocol = window.location.protocol === "https:" ? "https:" : "http:";
  const hostname = window.location.hostname || "127.0.0.1";
  return `${protocol}//${hostname}:8080`;
};

const masterApiBaseUrl = resolveMasterApiBaseUrl();
const closeSiteEndpoint = `${masterApiBaseUrl}/api/close-site`;

const clampPercent = (value: number) => Math.max(0, Math.min(100, value));
const clampUnit = (value: number) => Math.max(0, Math.min(1, value));
const requestClass = (requestId: string) => `req_id-${requestId}`;
const buildRequestEndpoint = (requestId: string) =>
  `${masterApiBaseUrl}/api/request/${encodeURIComponent(requestId)}`;
const buildRequestStatusEndpoint = (requestId: string) =>
  `${masterApiBaseUrl}/api/request-status/${encodeURIComponent(requestId)}`;
const buildServoRequestId = (baseRequestId: string, servoNumber: number) =>
  `${baseRequestId}_${servoNumber}`;
const buildKranActionRequestId = (baseRequestId: string, slot: number) =>
  `${baseRequestId}_${slot}`;
const buildTelemetryBoomSaveRequestId = (slot: number, mm: number) =>
  `${requestIds.telemetryBoomWrite}_${slot}_${mm}`;

const mapRectangles = [
  { id: "rectangle-0", label: "Позиция 1" },
  { id: "rectangle-1", label: "Позиция 2" },
  { id: "rectangle-2", label: "Позиция 3" },
  { id: "rectangle-3", label: "Позиция 4" },
] as const;

type MapRectangleId = (typeof mapRectangles)[number]["id"];

type RequestPayload = {
  ok?: boolean;
  response?: string;
  responses?: string[];
  error?: string;
  awaitingSecondResponse?: boolean;
  boomSlots?: unknown;
  systemSummary?: Partial<{
    robot: number;
    pre: number;
    crsf: number;
    batt: number;
    voltage_cV: number;
  }>;
};

const autonomyVariantsByRectangleId: Record<MapRectangleId, readonly string[]> =
  {
    "rectangle-0": ["Вариация автономки 1"],
    "rectangle-1": ["Вариация автономки 1", "Вариация автономки 2"],
    "rectangle-2": ["Вариация автономки 1"],
    "rectangle-3": ["Вариация автономки 1", "Вариация автономки 2"],
  };

const clampStatusLevel = (value: number): SystemStatusLevel => {
  if (value === 0) {
    return 0;
  }
  if (value === 2) {
    return 2;
  }
  return 1;
};

const parseSystemSummary = (
  summary: RequestPayload["systemSummary"] | undefined,
): SystemSummary | null => {
  if (!summary) {
    return null;
  }

  if (
    typeof summary.robot !== "number" ||
    typeof summary.pre !== "number" ||
    typeof summary.crsf !== "number" ||
    typeof summary.batt !== "number" ||
    typeof summary.voltage_cV !== "number"
  ) {
    return null;
  }

  return {
    robot: clampStatusLevel(summary.robot),
    pre: clampStatusLevel(summary.pre),
    crsf: clampStatusLevel(summary.crsf),
    batt: clampStatusLevel(summary.batt),
    voltage_cV: Math.max(0, Math.round(summary.voltage_cV)),
  };
};

const batteryPercentFromVoltage = (voltage_cV: number) =>
  clampPercent(
    ((voltage_cV - batteryEmptyCv) / (batteryFullCv - batteryEmptyCv)) * 100,
  );

const statusLevelClassName = (level: SystemStatusLevel) => {
  if (level === 2) {
    return "is-green";
  }
  if (level === 1) {
    return "is-yellow";
  }
  return "is-red";
};

function App() {
  const [activeTab, setActiveTab] = useState<TabId>("main");
  const [kranMode, setKranMode] = useState<KranMode>("manual");
  const [kranAction, setKranAction] = useState<KranAction>("home");
  const [selectedKranPosition, setSelectedKranPosition] = useState(0);
  const [isKranBusy, setIsKranBusy] = useState(false);
  const [selectedPawId, setSelectedPawId] = useState<1 | 2 | 3>(1);
  const [selectedServoNumber, setSelectedServoNumber] = useState<number>(
    servoOptions[0].servoNumber,
  );
  const [isServoMenuOpen, setIsServoMenuOpen] = useState(false);
  const [selectedTelemetryKey, setSelectedTelemetryKey] =
    useState<TelemetryOptionKey>(telemetryOptions[0].key);
  const [isTelemetryMenuOpen, setIsTelemetryMenuOpen] = useState(false);
  const [selectedTelemetryBoomSlot, setSelectedTelemetryBoomSlot] = useState(0);
  const [isTelemetryBoomSlotMenuOpen, setIsTelemetryBoomSlotMenuOpen] =
    useState(false);
  const [telemetryBoomPositionsMm, setTelemetryBoomPositionsMm] = useState<
    number[] | null
  >(null);
  const [telemetryBoomLoadState, setTelemetryBoomLoadState] = useState<
    "idle" | "loading" | "ready" | "error"
  >("idle");
  const [telemetryBoomErrorMessage, setTelemetryBoomErrorMessage] =
    useState("");
  const [telemetryBoomDraftMm, setTelemetryBoomDraftMm] = useState("0");
  const [isTelemetryBoomKeypadOpen, setIsTelemetryBoomKeypadOpen] =
    useState(false);
  const [requestState, setRequestState] = useState<RequestState>("idle");
  const [requestMessage, setRequestMessage] = useState("Ожидание команды");
  const [lastRequestId, setLastRequestId] = useState("");
  const [lastSlaveResponse, setLastSlaveResponse] = useState("");
  const [systemSummary, setSystemSummary] =
    useState<SystemSummary>(defaultSystemSummary);
  const [isSystemSummaryStale, setIsSystemSummaryStale] = useState(false);
  const [isDesktopActionPending, setIsDesktopActionPending] = useState(false);
  const [desktopActionMessage, setDesktopActionMessage] = useState("");
  const [isSecondResponseOverlayVisible, setIsSecondResponseOverlayVisible] =
    useState(false);
  const [secondResponseOverlayMessage, setSecondResponseOverlayMessage] =
    useState("");
  const [activeMapRectangleId, setActiveMapRectangleId] =
    useState<MapRectangleId>("rectangle-0");
  const [isAutonomyPositionMenuOpen, setIsAutonomyPositionMenuOpen] =
    useState(false);
  const [
    selectedAutonomyVariantByRectangleId,
    setSelectedAutonomyVariantByRectangleId,
  ] = useState<Record<MapRectangleId, number>>({
    "rectangle-0": 0,
    "rectangle-1": 0,
    "rectangle-2": 0,
    "rectangle-3": 0,
  });
  const [lapaScrollProgress, setLapaScrollProgress] = useState(0);
  const [isLapaGuideDragging, setIsLapaGuideDragging] = useState(false);

  const lapaScrollRef = useRef<HTMLDivElement | null>(null);
  const lapaGuideRef = useRef<HTMLDivElement | null>(null);
  const autonomyDropdownRef = useRef<HTMLDivElement | null>(null);
  const servoDropdownRef = useRef<HTMLDivElement | null>(null);
  const telemetryDropdownRef = useRef<HTMLDivElement | null>(null);
  const telemetryBoomSlotDropdownRef = useRef<HTMLDivElement | null>(null);
  const audioElementsRef = useRef<HTMLAudioElement[]>([]);
  const audioElementIndexRef = useRef(0);
  const requestInFlightRef = useRef(false);
  const sendSlaveRequestRef = useRef<
    | ((
        requestId: string,
        options?: SendSlaveRequestOptions,
      ) => Promise<SendSlaveRequestResult>)
    | null
  >(null);

  const selectedServoOption =
    servoOptions.find((option) => option.servoNumber === selectedServoNumber) ??
    servoOptions[0];
  const selectedTelemetryOption =
    telemetryOptions.find((option) => option.key === selectedTelemetryKey) ??
    telemetryOptions[0];
  const selectedTelemetryBoomSlotOption =
    telemetryBoomSlotOptions.find(
      (option) => option.slot === selectedTelemetryBoomSlot,
    ) ?? telemetryBoomSlotOptions[0];
  const selectedTelemetryBoomValueMm =
    telemetryBoomPositionsMm?.[selectedTelemetryBoomSlot] ?? 0;
  const normalizedTelemetryBoomDraftMm = normalizeTelemetryMmDraft(
    telemetryBoomDraftMm,
  );
  const hasTelemetryBoomDraftChanges =
    telemetryBoomPositionsMm !== null &&
    Number(normalizedTelemetryBoomDraftMm) !== selectedTelemetryBoomValueMm;
  const isTelemetryBoomSelected = selectedTelemetryOption.mode === "boomEditor";
  const telemetryBoomHasLoadedSlots = telemetryBoomPositionsMm !== null;
  const selectedAutonomyPosition =
    mapRectangles.find((rectangle) => rectangle.id === activeMapRectangleId) ??
    mapRectangles[0];
  const selectedAutonomyVariants =
    autonomyVariantsByRectangleId[activeMapRectangleId];
  const selectedAutonomyVariantIndex =
    selectedAutonomyVariantByRectangleId[activeMapRectangleId] ?? 0;
  const batteryPercent = batteryPercentFromVoltage(systemSummary.voltage_cV);
  const batteryStrokeColor =
    systemSummary.batt === 2
      ? "#0dff00"
      : systemSummary.batt === 1
        ? "#ffb020"
        : "#ff4f4f";
  const batteryTrackColor =
    systemSummary.batt === 2
      ? "rgba(13, 255, 0, 0.18)"
      : systemSummary.batt === 1
        ? "rgba(255, 176, 32, 0.18)"
        : "rgba(255, 79, 79, 0.18)";

  useEffect(() => {
    const updateScale = () => {
      const nextScale = Math.min(
        window.innerWidth / 1024,
        window.innerHeight / 600,
      );
      document.documentElement.style.setProperty(
        "--controlstation-scale",
        String(nextScale),
      );
    };

    updateScale();
    window.addEventListener("resize", updateScale);
    return () => {
      window.removeEventListener("resize", updateScale);
    };
  }, []);

  useEffect(() => {
    if (
      !isAutonomyPositionMenuOpen &&
      !isServoMenuOpen &&
      !isTelemetryMenuOpen &&
      !isTelemetryBoomSlotMenuOpen
    ) {
      return;
    }

    const handlePointerDown = (event: PointerEvent) => {
      const target = event.target as Node | null;
      if (!target) {
        return;
      }

      if (
        autonomyDropdownRef.current?.contains(target) ||
        servoDropdownRef.current?.contains(target) ||
        telemetryDropdownRef.current?.contains(target) ||
        telemetryBoomSlotDropdownRef.current?.contains(target)
      ) {
        return;
      }

      setIsAutonomyPositionMenuOpen(false);
      setIsServoMenuOpen(false);
      setIsTelemetryMenuOpen(false);
      setIsTelemetryBoomSlotMenuOpen(false);
    };

    document.addEventListener("pointerdown", handlePointerDown);
    return () => {
      document.removeEventListener("pointerdown", handlePointerDown);
    };
  }, [
    isAutonomyPositionMenuOpen,
    isServoMenuOpen,
    isTelemetryMenuOpen,
    isTelemetryBoomSlotMenuOpen,
  ]);

  useEffect(() => {
    setTelemetryBoomDraftMm(String(selectedTelemetryBoomValueMm));
  }, [selectedTelemetryBoomSlot, selectedTelemetryBoomValueMm]);

  useEffect(() => {
    if (!isTelemetryBoomKeypadOpen) {
      return;
    }

    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key !== "Escape") {
        return;
      }

      setTelemetryBoomDraftMm(String(selectedTelemetryBoomValueMm));
      setIsTelemetryBoomKeypadOpen(false);
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [isTelemetryBoomKeypadOpen, selectedTelemetryBoomValueMm]);

  const syncLapaScrollProgress = (element: HTMLDivElement) => {
    const maxScrollTop = element.scrollHeight - element.clientHeight;
    setLapaScrollProgress(
      maxScrollTop > 0 ? element.scrollTop / maxScrollTop : 0,
    );
  };

  const setLapaScrollByProgress = (nextProgress: number) => {
    const normalizedProgress = clampUnit(nextProgress);
    setLapaScrollProgress(normalizedProgress);

    const element = lapaScrollRef.current;
    if (!element) {
      return;
    }

    const maxScrollTop = element.scrollHeight - element.clientHeight;
    element.scrollTop =
      maxScrollTop > 0 ? maxScrollTop * normalizedProgress : 0;
  };

  const updateLapaGuideFromClientY = (clientY: number) => {
    const guideElement = lapaGuideRef.current;
    if (!guideElement) {
      return;
    }

    const { top } = guideElement.getBoundingClientRect();
    setLapaScrollByProgress((clientY - top - 8) / lapaGuideTravel);
  };

  const wait = (timeoutMs: number) =>
    new Promise<void>((resolve) => {
      window.setTimeout(resolve, timeoutMs);
    });

  const markSystemSummaryStale = () => {
    setIsSystemSummaryStale(true);
    setSystemSummary((currentSummary) => ({
      ...currentSummary,
      robot: 0,
      pre: 0,
      crsf: 0,
      batt: 0,
      voltage_cV: 0,
    }));
  };

  const awaitSecondSlaveResponse = async (
    requestId: string,
    options: SendSlaveRequestOptions = {},
  ): Promise<SendSlaveRequestResult> => {
    while (true) {
      await wait(250);

      const response = await fetch(buildRequestStatusEndpoint(requestId), {
        method: "GET",
      });
      const payload = (await response.json().catch(() => null)) as
        | RequestPayload
        | null;
      const parsedSummary = parseSystemSummary(payload?.systemSummary);
      if (parsedSummary) {
        setIsSystemSummaryStale(false);
        setSystemSummary(parsedSummary);
      }

      const slaveResponses = Array.isArray(payload?.responses)
        ? payload.responses
            .filter((value): value is string => typeof value === "string")
            .map((value) => value.trim())
            .filter((value) => value.length > 0)
        : [];
      const slaveResponse =
        slaveResponses.length > 0
          ? slaveResponses.join(" | ")
          : typeof payload?.response === "string"
            ? payload.response.trim()
            : "";

      if (!response.ok || !payload?.ok) {
        const errorMessage = payload?.error ?? `HTTP ${response.status}`;

        if (!options.isKranAction) {
          setIsSecondResponseOverlayVisible(false);
          setSecondResponseOverlayMessage("");
        }
        if (
          options.isKranAction &&
          shouldResetKranActionToHomeOnFailure(requestId)
        ) {
          setKranAction("home");
        }
        if (!options.isAutoRequest && !options.isSilent) {
          setRequestState("error");
          setRequestMessage(errorMessage);
          setLastSlaveResponse(slaveResponse);
        }
        return {
          ok: false,
          payload,
          errorMessage,
          slaveResponse,
          slaveResponses,
        };
      }

      if (payload.awaitingSecondResponse) {
        if (!options.isKranAction) {
          setSecondResponseOverlayMessage("Ожидание второго ответа от slave...");
        }
        continue;
      }

      if (!options.isKranAction) {
        setIsSecondResponseOverlayVisible(false);
        setSecondResponseOverlayMessage("");
      }
      if (
        options.isKranAction &&
        shouldResetKranActionToHomeOnSuccess(requestId)
      ) {
        setKranAction("home");
      }
      if (!options.isAutoRequest && !options.isSilent) {
        setRequestState("success");
        setRequestMessage(
          slaveResponses.length > 1
            ? `Получено ${slaveResponses.length} ответа на id ${requestId}`
            : `Получен второй ответ на id ${requestId}`,
        );
        setLastSlaveResponse(slaveResponse);
      }
      return {
        ok: true,
        payload,
        errorMessage: "",
        slaveResponse,
        slaveResponses,
      };
    }
  };

  const playButtonSound = () => {
    if (!audioElementsRef.current || audioElementsRef.current.length === 0) {
      return;
    }

    try {
      // Получаем следующий audio элемент по очереди
      const audioElement = audioElementsRef.current[audioElementIndexRef.current];
      audioElementIndexRef.current =
        (audioElementIndexRef.current + 1) % audioElementsRef.current.length;

      if (!audioElement) {
        return;
      }

      // Сбрасываем позицию и воспроизводим
      audioElement.currentTime = 0;
      const playPromise = audioElement.play();

      if (playPromise !== undefined) {
        playPromise.catch((err) => {
          console.debug("Failed to play button sound:", err);
        });
      }
    } catch (error) {
      console.debug("Failed to play button sound:", error);
    }
  };

  const sendSlaveRequest = async (
    requestId: string,
    options: SendSlaveRequestOptions = {},
  ): Promise<SendSlaveRequestResult> => {
    const {
      isAutoRequest = false,
      isKranAction = false,
      isSilent = false,
    } = options;

    if (requestInFlightRef.current) {
      return {
        ok: false,
        payload: null,
        errorMessage: "Request already in flight",
        slaveResponse: "",
        slaveResponses: [],
      };
    }

    requestInFlightRef.current = true;
    if (isKranAction) {
      setIsKranBusy(true);
    }

    // Play button sound on any button click (except auto requests)
    if (!isAutoRequest) {
      playButtonSound();
    }

    if (!isAutoRequest && !isSilent) {
      setRequestState("pending");
      setRequestMessage(`Отправка REQ,${requestId} на slave...`);
      setLastRequestId(requestId);
      setLastSlaveResponse("");
      if (!isKranAction) {
        setIsSecondResponseOverlayVisible(false);
        setSecondResponseOverlayMessage("");
      }
    }

    try {
      const response = await fetch(buildRequestEndpoint(requestId), {
        method: "POST",
      });
      const payload = (await response.json().catch(() => null)) as
        | RequestPayload
        | null;
      const parsedSummary = parseSystemSummary(payload?.systemSummary);
      if (parsedSummary) {
        setIsSystemSummaryStale(false);
        setSystemSummary(parsedSummary);
      }
      const slaveResponses = Array.isArray(payload?.responses)
        ? payload.responses
            .filter((value): value is string => typeof value === "string")
            .map((value) => value.trim())
            .filter((value) => value.length > 0)
        : [];
      const slaveResponse =
        slaveResponses.length > 0
          ? slaveResponses.join(" | ")
          : typeof payload?.response === "string"
            ? payload.response.trim()
            : "";

      if (!response.ok || !payload?.ok) {
        const errorMessage = payload?.error ?? `HTTP ${response.status}`;

        if (isKranAction && shouldResetKranActionToHomeOnFailure(requestId)) {
          setKranAction("home");
        }
        if (isAutoRequest) {
          markSystemSummaryStale();
        } else if (!isSilent) {
          setRequestState("error");
          setRequestMessage(errorMessage);
          setLastSlaveResponse(slaveResponse);
        }
        return {
          ok: false,
          payload,
          errorMessage,
          slaveResponse,
          slaveResponses,
        };
      }

      if (!isAutoRequest && !isSilent) {
        setRequestState("success");
        setLastSlaveResponse(slaveResponse);
      }

      if (payload?.awaitingSecondResponse) {
        if (!isAutoRequest && !isSilent) {
          setRequestMessage(`Получен первый ответ на id ${requestId}`);
        }
        if (!isKranAction) {
          setIsSecondResponseOverlayVisible(true);
          setSecondResponseOverlayMessage("Ожидание второго ответа от slave...");
        }
        return await awaitSecondSlaveResponse(requestId, options);
      }

      if (!isAutoRequest && !isSilent) {
        setRequestMessage(
          slaveResponses.length > 1
            ? `Получено ${slaveResponses.length} ответа на id ${requestId}`
            : `Ответ на id ${requestId} получен`,
        );
      }
      if (isKranAction && shouldResetKranActionToHomeOnSuccess(requestId)) {
        setKranAction("home");
      }
      return {
        ok: true,
        payload,
        errorMessage: "",
        slaveResponse,
        slaveResponses,
      };
    } catch (error) {
      if (isAutoRequest) {
        markSystemSummaryStale();
      } else {
        if (isKranAction && shouldResetKranActionToHomeOnFailure(requestId)) {
          setKranAction("home");
        }
        const errorMessage =
          error instanceof Error
            ? `Нет связи с мастером: ${error.message}`
            : "Нет связи с мастером";
        if (!isSilent) {
          setRequestState("error");
          setRequestMessage(errorMessage);
          setLastSlaveResponse("");
        }
        if (!isKranAction) {
          setIsSecondResponseOverlayVisible(false);
          setSecondResponseOverlayMessage("");
        }
        return {
          ok: false,
          payload: null,
          errorMessage,
          slaveResponse: "",
          slaveResponses: [],
        };
      }
      return {
        ok: false,
        payload: null,
        errorMessage: "System summary request failed",
        slaveResponse: "",
        slaveResponses: [],
      };
    } finally {
      if (isKranAction) {
        setIsKranBusy(false);
      }
      requestInFlightRef.current = false;
    }
  };

  const waitForRequestSlot = async () => {
    const deadline = performance.now() + kranActionRequestStartWaitMs;

    while (
      requestInFlightRef.current &&
      performance.now() < deadline
    ) {
      await wait(kranActionRequestRetryMs);
    }

    return !requestInFlightRef.current;
  };

  const startKranActionRequest = (nextAction: KranAction, requestId: string) => {
    void (async () => {
      if (!(await waitForRequestSlot()) || requestInFlightRef.current) {
        return;
      }

      setKranAction(nextAction);
      await sendSlaveRequest(requestId, {
        isKranAction: true,
      });
    })();
  };

  sendSlaveRequestRef.current = sendSlaveRequest;

  useEffect(() => {
    const pollIntervalMs = isSystemSummaryStale
      ? systemSummaryReconnectPollMs
      : systemSummaryPollMs;

    const pollSystemSummary = () => {
      if (requestInFlightRef.current) {
        return;
      }

      void sendSlaveRequestRef.current?.(requestIds.tabMain, {
        isAutoRequest: true,
      });
    };

    pollSystemSummary();
    const intervalId = window.setInterval(pollSystemSummary, pollIntervalMs);

    return () => {
      window.clearInterval(intervalId);
    };
  }, [isSystemSummaryStale]);

  const handleDesktopButtonClick = async () => {
    if (isDesktopActionPending) {
      return;
    }

    setIsDesktopActionPending(true);
    setDesktopActionMessage("Остановка сайта и Chromium...");

    try {
      const response = await fetch(closeSiteEndpoint, {
        method: "POST",
        keepalive: true,
      });
      const payload = (await response.json().catch(() => null)) as {
        ok?: boolean;
        response?: string;
        error?: string;
      } | null;

      if (!response.ok || !payload?.ok) {
        setDesktopActionMessage(payload?.error ?? `HTTP ${response.status}`);
        return;
      }

      setDesktopActionMessage("Команда на остановку отправлена");
    } catch (error) {
      setDesktopActionMessage(
        error instanceof Error
          ? `Не удалось остановить сайт: ${error.message}`
          : "Не удалось остановить сайт",
      );
    } finally {
      setIsDesktopActionPending(false);
    }
  };

  const handleTabClick = (tabId: TabId, requestId: string) => {
    setActiveTab(tabId);
    setIsServoMenuOpen(false);
    setIsTelemetryMenuOpen(false);
    setIsTelemetryBoomSlotMenuOpen(false);
    setIsTelemetryBoomKeypadOpen(false);
    void sendSlaveRequest(requestId);
  };

  const handleStaticRequestClick = (requestId: string, action?: () => void) => {
    action?.();
    void sendSlaveRequest(requestId);
  };

  const handleServoCommandClick = (
    baseRequestId: string,
    action?: () => void,
  ) => {
    action?.();
    void sendSlaveRequest(
      buildServoRequestId(baseRequestId, selectedServoNumber),
    );
  };

  const handleServoOptionClick = (servoNumber: number) => {
    setSelectedServoNumber(servoNumber);
    setIsServoMenuOpen(false);
    void sendSlaveRequest(
      buildServoRequestId(requestIds.servoSelect, servoNumber),
    );
  };

  const handleTelemetryOptionClick = (
    telemetryKey: TelemetryOptionKey,
    requestId?: string,
  ) => {
    setSelectedTelemetryKey(telemetryKey);
    setIsTelemetryMenuOpen(false);
    setIsTelemetryBoomSlotMenuOpen(false);
    setIsTelemetryBoomKeypadOpen(false);
    if (telemetryKey !== "boom") {
      setTelemetryBoomErrorMessage("");
    }
    if (requestId) {
      void sendSlaveRequest(requestId);
    }
  };

  const handleTelemetryBoomSlotClick = (slot: number) => {
    setSelectedTelemetryBoomSlot(slot);
    setIsTelemetryBoomSlotMenuOpen(false);
    setIsTelemetryBoomKeypadOpen(false);
  };

  const handleTelemetryBoomDigitClick = (digit: string) => {
    setIsTelemetryBoomKeypadOpen(true);
    if (digit === "C") {
      setTelemetryBoomDraftMm("0");
      return;
    }

    if (digit === "⌫") {
      setTelemetryBoomDraftMm((currentValue) =>
        backspaceTelemetryMmDraft(currentValue),
      );
      return;
    }

    setTelemetryBoomDraftMm((currentValue) =>
      appendDigitToTelemetryMmDraft(currentValue, digit),
    );
  };

  const loadTelemetryBoomSlots = async () => {
    if (!(await waitForRequestSlot()) || requestInFlightRef.current) {
      setTelemetryBoomPositionsMm(null);
      setTelemetryBoomLoadState("error");
      setTelemetryBoomErrorMessage("Канал обмена занят");
      return null;
    }

    setTelemetryBoomLoadState("loading");
    setTelemetryBoomErrorMessage("");

    const result = await sendSlaveRequest(requestIds.telemetryBoomRead, {
      isSilent: true,
    });
    const parsedSlots = parseTelemetryBoomSlots(result.payload?.boomSlots);
    if (!result.ok || parsedSlots === null) {
      setTelemetryBoomPositionsMm(null);
      setTelemetryBoomLoadState("error");
      setTelemetryBoomErrorMessage(
        result.errorMessage || "Не удалось прочитать позиции стрелы",
      );
      return null;
    }

    setTelemetryBoomPositionsMm(parsedSlots);
    setTelemetryBoomLoadState("ready");
    setTelemetryBoomErrorMessage("");
    return parsedSlots;
  };

  const closeTelemetryBoomKeypad = () => {
    setTelemetryBoomDraftMm(String(selectedTelemetryBoomValueMm));
    setIsTelemetryBoomKeypadOpen(false);
  };

  const handleTelemetryBoomSave = () => {
    void (async () => {
      if (!telemetryBoomHasLoadedSlots) {
        return;
      }

      const nextValueMm = Number(normalizedTelemetryBoomDraftMm);
      if (!isTelemetryBoomMmInRange(nextValueMm)) {
        setTelemetryBoomLoadState("error");
        setTelemetryBoomErrorMessage(
          `Допустимый диапазон ${telemetryBoomMinMm}..${telemetryBoomMaxMm} мм`,
        );
        return;
      }

      if (!(await waitForRequestSlot()) || requestInFlightRef.current) {
        setTelemetryBoomLoadState("error");
        setTelemetryBoomErrorMessage("Канал обмена занят");
        return;
      }

      setTelemetryBoomLoadState("loading");
      const result = await sendSlaveRequest(
        buildTelemetryBoomSaveRequestId(selectedTelemetryBoomSlot, nextValueMm),
        { isSilent: true },
      );
      if (!result.ok) {
        setTelemetryBoomLoadState("error");
        setTelemetryBoomErrorMessage(
          result.errorMessage || "Не удалось сохранить позицию стрелы",
        );
        return;
      }

      setIsTelemetryBoomKeypadOpen(false);
      setTelemetryBoomDraftMm(String(nextValueMm));
      await loadTelemetryBoomSlots();
    })();
  };

  useEffect(() => {
    if (
      !shouldClearTelemetryBoomSlots({
        isTelemetryTabActive: activeTab === "telemetry",
        isBoomEditorSelected: isTelemetryBoomSelected,
        isSystemSummaryStale,
      })
    ) {
      return;
    }

    setTelemetryBoomPositionsMm(null);
    setTelemetryBoomLoadState("idle");
    setTelemetryBoomErrorMessage("");
    setIsTelemetryBoomKeypadOpen(false);
  }, [activeTab, isTelemetryBoomSelected, isSystemSummaryStale]);

  useEffect(() => {
    if (
      !shouldLoadTelemetryBoomSlots({
        isTelemetryTabActive: activeTab === "telemetry",
        isBoomEditorSelected: isTelemetryBoomSelected,
        isSystemSummaryStale,
      })
    ) {
      return;
    }

    void loadTelemetryBoomSlots();
  }, [activeTab, isTelemetryBoomSelected, isSystemSummaryStale]);

  const requestFeedbackClassName =
    requestState === "success" ? "is-success" : "";

  const renderStatusContent = () => {
    if (activeTab === "main") {
      return (
        <div className="status-panel__main">
          <div className="status-panel__aside">
            <div
              className="status-card glass-panel"
              aria-label="Состояние системы"
            >
              <h1 className="status-card__title">Состояние системы</h1>
              <ul className="status-card__list">
                {systemStatuses.map((statusItem) => (
                  <li key={statusItem.key} className="status-card__item">
                    <span
                      className={`status-card__dot ${statusLevelClassName(systemSummary[statusItem.key])}`.trim()}
                      aria-hidden="true"
                    />
                    <span>{statusItem.label}</span>
                  </li>
                ))}
              </ul>
            </div>

            <section
              className="autonomy-card glass-panel"
              aria-label="Выбор автономной позиции"
            >
              <h2 className="autonomy-card__title">Автономка</h2>

              <div ref={autonomyDropdownRef} className="autonomy-dropdown">
                <button
                  type="button"
                  className={`autonomy-dropdown__trigger glass-panel ${isAutonomyPositionMenuOpen ? "is-open" : ""}`.trim()}
                  aria-expanded={isAutonomyPositionMenuOpen}
                  aria-haspopup="listbox"
                  onClick={() => {
                    setIsAutonomyPositionMenuOpen(
                      (currentValue) => !currentValue,
                    );
                  }}
                >
                  <span>{selectedAutonomyPosition.label}</span>
                  <span className="autonomy-dropdown__arrow" aria-hidden="true">
                    ⌄
                  </span>
                </button>

                {isAutonomyPositionMenuOpen ? (
                  <div
                    className="autonomy-dropdown__menu glass-panel"
                    role="listbox"
                    aria-label="Список позиций"
                  >
                    {mapRectangles.map((rectangle) => (
                      <button
                        key={rectangle.id}
                        type="button"
                        className={`autonomy-dropdown__option ${activeMapRectangleId === rectangle.id ? "is-selected" : ""}`.trim()}
                        role="option"
                        aria-selected={activeMapRectangleId === rectangle.id}
                        onClick={() => {
                          setActiveMapRectangleId(rectangle.id);
                          setIsAutonomyPositionMenuOpen(false);
                        }}
                      >
                        {rectangle.label}
                      </button>
                    ))}
                  </div>
                ) : null}
              </div>

              <ul
                className="autonomy-card__variants"
                aria-label="Вариации автономки"
              >
                {selectedAutonomyVariants.map((variant, variantIndex) => (
                  <li
                    key={`${activeMapRectangleId}-${variant}`}
                    className="status-card__item"
                  >
                    <button
                      type="button"
                      className={`autonomy-card__variant ${selectedAutonomyVariantIndex === variantIndex ? "is-selected" : ""}`.trim()}
                      onClick={() => {
                        setSelectedAutonomyVariantByRectangleId(
                          (currentValue) => ({
                            ...currentValue,
                            [activeMapRectangleId]: variantIndex,
                          }),
                        );
                      }}
                    >
                      <span
                        className={`status-card__dot ${selectedAutonomyVariantIndex === variantIndex ? "" : "is-muted"}`.trim()}
                        aria-hidden="true"
                      />
                      <span>{variant}</span>
                    </button>
                  </li>
                ))}
              </ul>
            </section>
          </div>

          <div className="map-card glass-panel" aria-label="Карта площадки">
            <img
              className="map-card__image"
              src={mapImage}
              alt="Карта площадки"
            />

            {mapRectangles.map((rectangle) => (
              <button
                key={rectangle.id}
                type="button"
                className={`map-card__active-position ${rectangle.id} ${activeMapRectangleId === rectangle.id ? "active" : ""}`.trim()}
                onClick={() => {
                  setActiveMapRectangleId(rectangle.id);
                }}
                aria-label={rectangle.label}
                aria-pressed={activeMapRectangleId === rectangle.id}
              >
                <img src={activePositionImage} alt="" aria-hidden="true" />
              </button>
            ))}
          </div>
        </div>
      );
    }

    if (activeTab === "kran") {
      return (
        <div className="kran-panel" aria-label="Вкладка крана">
          <div
            className="kran-preview glass-panel"
            aria-label="Просмотр крана"
          />

          <div className="kran-controls">
            <section
              className="kran-mode-card glass-panel"
              aria-label="Текущий режим"
            >
              <div className="kran-mode-card__header">
                <h1 className="kran-mode-card__title">Текущий режим</h1>

                <div className="kran-mode-card__panel glass-panel">
                  <div className="kran-mode-card__switch">
                    <button
                      type="button"
                      className={`kran-mode-card__mode ${requestClass(requestIds.kranModeAuto)} ${kranMode === "auto" ? "is-active" : ""}`.trim()}
                      data-request-id={requestIds.kranModeAuto}
                      aria-pressed={kranMode === "auto"}
                      disabled={isKranBusy}
                      onClick={() => {
                        handleStaticRequestClick(
                          requestIds.kranModeAuto,
                          () => {
                            setKranMode("auto");
                          },
                        );
                      }}
                    >
                      Авто
                    </button>

                    <button
                      type="button"
                      className={`kran-mode-card__mode ${requestClass(requestIds.kranModeManual)} ${kranMode === "manual" ? "is-active" : ""}`.trim()}
                      data-request-id={requestIds.kranModeManual}
                      aria-pressed={kranMode === "manual"}
                      disabled={isKranBusy}
                      onClick={() => {
                        handleStaticRequestClick(
                          requestIds.kranModeManual,
                          () => {
                            setKranMode("manual");
                          },
                        );
                      }}
                    >
                      Ручной
                    </button>
                  </div>
                </div>
              </div>

              <div className="kran-mode-card__selector">
                <span className="kran-mode-card__selector-label">
                  Текущая позиция
                </span>
                <div
                  className="kran-mode-card__selector-controls"
                  aria-label="Управление позицией крана"
                >
                  <span className="kran-mode-card__selector-value">
                    {selectedKranPosition}
                  </span>
                  <button
                    type="button"
                    className="kran-mode-card__arrow"
                    disabled={isKranBusy}
                    onClick={() => {
                      setSelectedKranPosition((currentValue) =>
                        Math.min(10, currentValue + 1),
                      );
                    }}
                  >
                    ⌃
                  </button>

                  <button
                    type="button"
                    className="kran-mode-card__arrow"
                    disabled={isKranBusy}
                    onClick={() => {
                      setSelectedKranPosition((currentValue) =>
                        Math.max(0, currentValue - 1),
                      );
                    }}
                  >
                    ⌄
                  </button>
                </div>
              </div>
            </section>

            <section
              className="kran-actions glass-panel"
              aria-label="Действия крана"
            >
              <button
                type="button"
                className={`kran-actions__secondary ${requestClass(requestIds.kranHome)} ${kranAction === "home" ? "is-active" : ""}`.trim()}
                data-request-id={buildKranActionRequestId(requestIds.kranHome, 0)}
                aria-pressed={kranAction === "home"}
                disabled={isKranBusy}
                onClick={() => {
                  startKranActionRequest(
                    "home",
                    buildKranActionRequestId(requestIds.kranHome, 0),
                  );
                }}
              >
                Исходное
              </button>

              <button
                type="button"
                className={`kran-actions__primary ${requestClass(requestIds.kranThrow)} ${kranAction === "throw" ? "is-active" : ""}`.trim()}
                data-request-id={buildKranActionRequestId(
                  requestIds.kranThrow,
                  selectedKranPosition,
                )}
                aria-pressed={kranAction === "throw"}
                disabled={isKranBusy}
                onClick={() => {
                  startKranActionRequest(
                    "throw",
                    buildKranActionRequestId(
                      requestIds.kranThrow,
                      selectedKranPosition,
                    ),
                  );
                }}
              >
                Выбросить
              </button>
            </section>
          </div>
        </div>
      );
    }

    if (activeTab === "lapa") {
      return (
        <div className="lapa-panel" aria-label="Вкладка лапы">
          <div>
            <h1 className="lapa-panel__title">Выбор лапы</h1>

            <div className="lapa-panel__tabs" aria-label="Выбор лапы">
              {[1, 2, 3].map((pawId) => {
                const requestId =
                  pawId === 1
                    ? requestIds.paw1
                    : pawId === 2
                      ? requestIds.paw2
                      : requestIds.paw3;

                return (
                  <button
                    key={pawId}
                    type="button"
                    className={`lapa-panel__tab ${requestClass(requestId)} ${selectedPawId === pawId ? "is-active" : ""}`.trim()}
                    data-request-id={requestId}
                    onClick={() => {
                      handleStaticRequestClick(requestId, () => {
                        setSelectedPawId(pawId as 1 | 2 | 3);
                      });
                    }}
                  >
                    Лапа {pawId}
                  </button>
                );
              })}
            </div>

            <div className="lapa-panel__actions" aria-label="Команды лапы">
              <button
                type="button"
                className={`lapa-panel__action ${requestClass(requestIds.pawCapture)}`.trim()}
                data-request-id={requestIds.pawCapture}
                onClick={() => {
                  handleStaticRequestClick(requestIds.pawCapture);
                }}
              >
                Захватить
              </button>

              <button
                type="button"
                className={`lapa-panel__action ${requestClass(requestIds.pawRelease)}`.trim()}
                data-request-id={requestIds.pawRelease}
                onClick={() => {
                  handleStaticRequestClick(requestIds.pawRelease);
                }}
              >
                Отпустить
              </button>
            </div>
          </div>

          <div
            className="lapa-preview lapa-preview--side glass-panel"
            aria-label="Боковой просмотр лапы"
          >
            <div
              ref={lapaGuideRef}
              className={`lapa-preview__guide ${isLapaGuideDragging ? "is-dragging" : ""}`.trim()}
              role="scrollbar"
              aria-label="Прокрутка бокового просмотра лапы"
              aria-controls="lapa-preview-scroll"
              aria-orientation="vertical"
              aria-valuemin={0}
              aria-valuemax={100}
              aria-valuenow={Math.round(lapaScrollProgress * 100)}
              tabIndex={0}
              onPointerDown={(event) => {
                setIsLapaGuideDragging(true);
                event.currentTarget.setPointerCapture(event.pointerId);
                updateLapaGuideFromClientY(event.clientY);
              }}
              onPointerMove={(event) => {
                if (isLapaGuideDragging) {
                  updateLapaGuideFromClientY(event.clientY);
                }
              }}
              onPointerUp={(event) => {
                setIsLapaGuideDragging(false);
                event.currentTarget.releasePointerCapture(event.pointerId);
              }}
              onPointerCancel={(event) => {
                setIsLapaGuideDragging(false);
                event.currentTarget.releasePointerCapture(event.pointerId);
              }}
              onKeyDown={(event) => {
                if (event.key === "ArrowDown") {
                  event.preventDefault();
                  setLapaScrollByProgress(lapaScrollProgress + 0.06);
                }

                if (event.key === "ArrowUp") {
                  event.preventDefault();
                  setLapaScrollByProgress(lapaScrollProgress - 0.06);
                }

                if (event.key === "Home") {
                  event.preventDefault();
                  setLapaScrollByProgress(0);
                }

                if (event.key === "End") {
                  event.preventDefault();
                  setLapaScrollByProgress(1);
                }
              }}
            >
              <span className="lapa-preview__guide-line" />
              <span
                className="lapa-preview__guide-knob"
                style={{ top: `${lapaScrollProgress * lapaGuideTravel}px` }}
              />
            </div>

            <div
              id="lapa-preview-scroll"
              ref={lapaScrollRef}
              className="lapa-preview__scroll"
              onScroll={(event) => {
                syncLapaScrollProgress(event.currentTarget);
              }}
            />
          </div>
        </div>
      );
    }

    if (activeTab === "servo") {
      return (
        <div className="lapa-panel" aria-label="Вкладка сервопривода">
          <div className="servo-choosing">
            <h1 className="servo-choosing__title">Выбор сервопривода</h1>

            <div
              ref={servoDropdownRef}
              className="telemetry-dropdown servo-dropdown"
            >
              <button
                type="button"
                className={`telemetry-dropdown__trigger servo-choosing__selector glass-panel ${requestClass(requestIds.servoDropdownToggle)} ${isServoMenuOpen ? "is-open" : ""}`.trim()}
                data-request-id={buildServoRequestId(
                  requestIds.servoDropdownToggle,
                  selectedServoNumber,
                )}
                data-servo-number={selectedServoNumber}
                aria-expanded={isServoMenuOpen}
                aria-haspopup="listbox"
                onClick={() => {
                  handleServoCommandClick(
                    requestIds.servoDropdownToggle,
                    () => {
                      setIsServoMenuOpen((currentValue) => !currentValue);
                    },
                  );
                }}
              >
                <span>{selectedServoOption.label}</span>
                <span
                  className="telemetry-dropdown__arrow servo-choosing__selector-arrow"
                  aria-hidden="true"
                >
                  ⌄
                </span>
              </button>

              {isServoMenuOpen ? (
                <div
                  className="telemetry-dropdown__menu glass-panel"
                  role="listbox"
                  aria-label="Список сервоприводов"
                >
                  {servoOptions.map((option) => (
                    <button
                      key={option.servoNumber}
                      type="button"
                      className={`telemetry-dropdown__option ${requestClass(requestIds.servoSelect)} ${selectedServoNumber === option.servoNumber ? "is-selected" : ""}`.trim()}
                      data-request-id={buildServoRequestId(
                        requestIds.servoSelect,
                        option.servoNumber,
                      )}
                      data-servo-number={option.servoNumber}
                      role="option"
                      aria-selected={selectedServoNumber === option.servoNumber}
                      onClick={() => {
                        handleServoOptionClick(option.servoNumber);
                      }}
                    >
                      {option.label}
                    </button>
                  ))}
                </div>
              ) : null}
            </div>

            <div className="servo-choosing__split-row">
              <button
                type="button"
                className={`servo-choosing__half-button glass-panel ${requestClass(requestIds.servoMinus)}`.trim()}
                data-request-id={buildServoRequestId(
                  requestIds.servoMinus,
                  selectedServoNumber,
                )}
                data-servo-number={selectedServoNumber}
                onClick={() => {
                  handleServoCommandClick(requestIds.servoMinus);
                }}
              >
                A-
              </button>

              <button
                type="button"
                className={`servo-choosing__half-button glass-panel ${requestClass(requestIds.servoPlus)}`.trim()}
                data-request-id={buildServoRequestId(
                  requestIds.servoPlus,
                  selectedServoNumber,
                )}
                data-servo-number={selectedServoNumber}
                onClick={() => {
                  handleServoCommandClick(requestIds.servoPlus);
                }}
              >
                A+
              </button>
            </div>

            <button
              type="button"
              className={`servo-choosing__wide-button glass-panel ${requestClass(requestIds.servoReturnZero)}`.trim()}
              data-request-id={buildServoRequestId(
                requestIds.servoReturnZero,
                selectedServoNumber,
              )}
              data-servo-number={selectedServoNumber}
              onClick={() => {
                handleServoCommandClick(requestIds.servoReturnZero);
              }}
            >
              Возврат в 0
            </button>

            <button
              type="button"
              className={`servo-choosing__wide-button glass-panel ${requestClass(requestIds.servoSetZero)}`.trim()}
              data-request-id={buildServoRequestId(
                requestIds.servoSetZero,
                selectedServoNumber,
              )}
              data-servo-number={selectedServoNumber}
              onClick={() => {
                handleServoCommandClick(requestIds.servoSetZero);
              }}
            >
              Задать 0 с текущего положения
            </button>

            <button
              type="button"
              className={`servo-choosing__wide-button servo-choosing__wide-button--multiline glass-panel ${requestClass(requestIds.servoSetEnd)}`.trim()}
              data-request-id={buildServoRequestId(
                requestIds.servoSetEnd,
                selectedServoNumber,
              )}
              data-servo-number={selectedServoNumber}
              onClick={() => {
                handleServoCommandClick(requestIds.servoSetEnd);
              }}
            >
              Задать конечную позицию с текущего положения
            </button>

            <div className="servo-choosing__readings">
              <div className="servo-choosing__reading">
                <span className="servo-choosing__reading-label">
                  Установленная начальная позиция
                </span>
                <span className="servo-choosing__reading-value glass-panel">
                  {selectedServoOption.initialValue}
                </span>
              </div>

              <div className="servo-choosing__reading">
                <span className="servo-choosing__reading-label">
                  Установленная конечная позиция
                </span>
                <span className="servo-choosing__reading-value glass-panel">
                  {selectedServoOption.finalValue}
                </span>
              </div>
            </div>

            <div className="servo-choosing__footer">
              <div className="servo-choosing__footer-value glass-panel">
                {selectedServoOption.currentValue}
              </div>

              <button
                type="button"
                className={`servo-choosing__send glass-panel ${requestClass(requestIds.servoSend)}`.trim()}
                data-request-id={buildServoRequestId(
                  requestIds.servoSend,
                  selectedServoNumber,
                )}
                data-servo-number={selectedServoNumber}
                onClick={() => {
                  handleServoCommandClick(requestIds.servoSend);
                }}
              >
                Отправить
              </button>
            </div>
          </div>

          <div
            className="lapa-preview lapa-preview--side glass-panel servo-preview"
            aria-label="Просмотр сервопривода"
          >
            <div className="lapa-preview__scroll servo-preview__scroll">
              <div className="servo-preview__footer glass-panel">
                <span className="servo-preview__footer-value">
                  {selectedServoOption.currentValue}
                </span>
              </div>
            </div>
          </div>
        </div>
      );
    }

    if (activeTab === "telemetry") {
      return (
        <div className="lapa-panel" aria-label="Вкладка телеметрии">
          <div className="telemetry-choosing">
            <h1 className="telemetry-choosing__title">
              Выбор узла для телеметрии
            </h1>

            <div ref={telemetryDropdownRef} className="telemetry-dropdown">
              <button
                type="button"
                className={`telemetry-dropdown__trigger glass-panel ${requestClass(requestIds.telemetryDropdownToggle)} ${isTelemetryMenuOpen ? "is-open" : ""}`.trim()}
                data-request-id={requestIds.telemetryDropdownToggle}
                aria-expanded={isTelemetryMenuOpen}
                aria-haspopup="listbox"
                onClick={() => {
                  handleStaticRequestClick(
                    requestIds.telemetryDropdownToggle,
                    () => {
                      setIsTelemetryMenuOpen((currentValue) => !currentValue);
                    },
                  );
                }}
              >
                <span>{selectedTelemetryOption.label}</span>
                <span className="telemetry-dropdown__arrow" aria-hidden="true">
                  ⌄
                </span>
              </button>

              {isTelemetryMenuOpen ? (
                <div
                  className="telemetry-dropdown__menu glass-panel"
                  role="listbox"
                  aria-label="Список узлов телеметрии"
                >
                  {telemetryOptions.map((option) => (
                    <button
                      key={option.key}
                      type="button"
                      className={`telemetry-dropdown__option ${option.requestId ? requestClass(option.requestId) : ""} ${selectedTelemetryKey === option.key ? "is-selected" : ""}`.trim()}
                      data-request-id={option.requestId}
                      role="option"
                      aria-selected={selectedTelemetryKey === option.key}
                      onClick={() => {
                        handleTelemetryOptionClick(
                          option.key,
                          option.requestId,
                        );
                      }}
                    >
                      {option.label}
                    </button>
                  ))}
                </div>
              ) : null}
            </div>

            {isTelemetryBoomSelected ? (
              <div className="telemetry-boom-editor">
                <div
                  ref={telemetryBoomSlotDropdownRef}
                  className="telemetry-dropdown telemetry-dropdown--compact"
                >
                  <span className="telemetry-choosing__metric-label">
                    Выбор позиции
                  </span>
                  <button
                    type="button"
                    className={`telemetry-dropdown__trigger telemetry-dropdown__trigger--compact glass-panel ${isTelemetryBoomSlotMenuOpen ? "is-open" : ""}`.trim()}
                    aria-expanded={isTelemetryBoomSlotMenuOpen}
                    aria-haspopup="listbox"
                    onClick={() => {
                      if (!telemetryBoomHasLoadedSlots) {
                        return;
                      }
                      setIsTelemetryBoomSlotMenuOpen((currentValue) => !currentValue);
                    }}
                  >
                    <span>{selectedTelemetryBoomSlotOption.label}</span>
                    <span className="telemetry-dropdown__arrow" aria-hidden="true">
                      ⌄
                    </span>
                  </button>

                  {isTelemetryBoomSlotMenuOpen ? (
                    <div
                      className="telemetry-dropdown__menu telemetry-dropdown__menu--scrollable glass-panel"
                      role="listbox"
                      aria-label="Список позиций стрелы"
                    >
                      {telemetryBoomSlotOptions.map((option) => (
                        <button
                          key={option.slot}
                          type="button"
                          className={`telemetry-dropdown__option ${selectedTelemetryBoomSlot === option.slot ? "is-selected" : ""}`.trim()}
                          role="option"
                          aria-selected={selectedTelemetryBoomSlot === option.slot}
                          onClick={() => {
                            handleTelemetryBoomSlotClick(option.slot);
                          }}
                        >
                          {option.label}
                        </button>
                      ))}
                    </div>
                  ) : null}
                </div>

                <div className="telemetry-boom-editor__value-block">
                  <span className="telemetry-choosing__metric-label">
                    Текущее значение, мм
                  </span>
                  <button
                    type="button"
                    className={`telemetry-boom-editor__value glass-panel ${isTelemetryBoomKeypadOpen ? "is-editing" : ""}`.trim()}
                    disabled={!telemetryBoomHasLoadedSlots}
                    onClick={() => {
                      if (!telemetryBoomHasLoadedSlots) {
                        return;
                      }
                      setTelemetryBoomDraftMm(String(selectedTelemetryBoomValueMm));
                      setIsTelemetryBoomKeypadOpen(true);
                    }}
                  >
                    {telemetryBoomHasLoadedSlots
                      ? `${selectedTelemetryBoomValueMm} мм`
                      : telemetryBoomLoadState === "loading"
                        ? "Загрузка..."
                        : "Нет данных"}
                  </button>
                </div>

                <p className="telemetry-boom-editor__hint">
                  Значения позиций читаются напрямую из `stepper`. Изменение
                  сохраняется в память контроллера после команды `Сохранить`.
                </p>

                {telemetryBoomErrorMessage ? (
                  <p className="telemetry-boom-editor__error">
                    {telemetryBoomErrorMessage}
                  </p>
                ) : null}
              </div>
            ) : (
              <>
                <div className="telemetry-choosing__metric-row">
                  <span className="telemetry-choosing__metric-label">
                    {selectedTelemetryOption.metricLabel}
                  </span>
                  <span className="telemetry-choosing__metric-value glass-panel">
                    {selectedTelemetryOption.metricValue}
                  </span>
                </div>

                <div className="telemetry-choosing__footer">
                  <div className="telemetry-choosing__footer-value glass-panel">
                    {selectedTelemetryOption.footerValue}
                  </div>
                </div>
              </>
            )}
          </div>

          <div
            className="lapa-preview lapa-preview--side glass-panel telemetry-preview"
            aria-label="Просмотр телеметрии"
          >
            {selectedTelemetryKey === "wheelbase" ? (
              <div className="telemetry-preview__scroll">
                <img
                  className="telemetry-preview__image"
                  src={wheelsImage}
                  alt="Колёсная база"
                />
              </div>
            ) : isTelemetryBoomSelected ? (
              <div className="telemetry-preview__slots">
                <h2 className="telemetry-preview__slots-title">Позиции стрелы</h2>
                {telemetryBoomHasLoadedSlots ? (
                  <div className="telemetry-preview__slots-list">
                    {telemetryBoomSlotOptions.map((option) => (
                      <button
                        key={option.slot}
                        type="button"
                        className={`telemetry-preview__slot glass-panel ${selectedTelemetryBoomSlot === option.slot ? "is-active" : ""}`.trim()}
                        onClick={() => {
                          handleTelemetryBoomSlotClick(option.slot);
                        }}
                      >
                        <span className="telemetry-preview__slot-label">
                          {option.label}
                        </span>
                        <span className="telemetry-preview__slot-value">
                          {telemetryBoomPositionsMm?.[option.slot] ?? 0} мм
                        </span>
                      </button>
                    ))}
                  </div>
                ) : (
                  <div className="telemetry-preview__empty">
                    {telemetryBoomLoadState === "loading"
                      ? "Чтение значений из stepper..."
                      : telemetryBoomErrorMessage || "Нет данных по позициям"}
                  </div>
                )}
              </div>
            ) : null}
          </div>
        </div>
      );
    }

    if (activeTab === "label") {
      return (
        <div className="status-panel__main">
        </div>
      );
    }

    return null;
  };

  return (
    <div className="app-shell">
      {[0, 1, 2, 3].map((index) => (
        <audio
          key={index}
          ref={(el) => {
            if (el) {
              audioElementsRef.current[index] = el;
            }
          }}
          src="/sounds/discord-notification.mp3"
          preload="auto"
          style={{ display: "none" }}
        />
      ))}
      <div className="app-background" aria-hidden="true">
        <img
          className="app-background__image"
          src={backgroundImage}
          alt=""
          aria-hidden="true"
        />
        <LiquidChrome
          baseColor={liquidChromeBaseColor}
          speed={0.12}
          amplitude={0.24}
          interactive={false}
          renderScale={0.35}
          maxFps={18}
        />
      </div>

      <main className="main-screen app-content">
        <div className="desktop-tools">
          <button
            type="button"
            className="desktop-tools__button glass-panel"
            onClick={() => {
              void handleDesktopButtonClick();
            }}
            title="Остановить сайт и Chromium"
            aria-labelledby="desktop-tools-button-label"
            disabled={isDesktopActionPending}
          >
            Закрыть сайт
          </button>
          <span id="desktop-tools-button-label" className="sr-only">
            Остановить сайт и Chromium
          </span>

          {requestState !== "idle" ? (
            <p
              className={`desktop-tools__feedback ${requestFeedbackClassName}`.trim()}
              aria-live="polite"
            >
              {lastRequestId ? `id: ${lastRequestId}. ` : ""}
              {requestMessage}
              {lastSlaveResponse ? ` Ответ slave: ${lastSlaveResponse}` : ""}
            </p>
          ) : null}

          {desktopActionMessage ? (
            <p className="desktop-tools__feedback is-error" aria-live="polite">
              {desktopActionMessage}
            </p>
          ) : null}
        </div>

        {isSecondResponseOverlayVisible ? (
          <div
            className="request-overlay"
            aria-live="assertive"
            aria-label="Ожидание второго ответа"
          >
            <div className="request-overlay__card glass-panel">
              <p className="request-overlay__title">Ожидание второго ответа</p>
              <p className="request-overlay__message">
                {secondResponseOverlayMessage}
              </p>
            </div>
          </div>
        ) : null}

        {activeTab === "telemetry" &&
        isTelemetryBoomSelected &&
        isTelemetryBoomKeypadOpen ? (
          <div
            className="telemetry-boom-modal"
            aria-live="polite"
            aria-label="Редактор расстояния стрелы"
            onClick={() => {
              closeTelemetryBoomKeypad();
            }}
          >
            <div
              className="telemetry-boom-modal__card glass-panel"
              role="dialog"
              aria-modal="true"
              aria-labelledby="telemetry-boom-modal-title"
              onClick={(event) => {
                event.stopPropagation();
              }}
            >
              <p
                id="telemetry-boom-modal-title"
                className="telemetry-boom-modal__title"
              >
                {selectedTelemetryBoomSlotOption.label}
              </p>
              <p className="telemetry-boom-modal__subtitle">
                Введите расстояние выдвижения в миллиметрах
                {` (${telemetryBoomMinMm}..${telemetryBoomMaxMm})`}
              </p>

              <div className="telemetry-boom-editor__keypad telemetry-boom-modal__keypad glass-panel">
                <div className="telemetry-boom-editor__draft">
                  {normalizedTelemetryBoomDraftMm} мм
                </div>

                <div className="telemetry-boom-editor__digits">
                  {telemetryBoomKeypadDigits.map((row, rowIndex) => (
                    <div key={rowIndex} className="telemetry-boom-editor__digit-row">
                      {row.map((digit) => (
                        <button
                          key={digit}
                          type="button"
                          className={`telemetry-boom-editor__digit ${digit === "C" ? "is-accent" : ""}`.trim()}
                          onClick={() => {
                            handleTelemetryBoomDigitClick(digit);
                          }}
                        >
                          {digit}
                        </button>
                      ))}
                    </div>
                  ))}
                </div>
              </div>

              <div className="telemetry-boom-modal__actions">
                <button
                  type="button"
                  className="telemetry-boom-modal__action glass-panel"
                  onClick={() => {
                    closeTelemetryBoomKeypad();
                  }}
                >
                  Отмена
                </button>
                <button
                  type="button"
                  className="telemetry-boom-editor__save telemetry-boom-modal__action glass-panel"
                  onClick={() => {
                    handleTelemetryBoomSave();
                  }}
                  disabled={!hasTelemetryBoomDraftChanges}
                >
                  Сохранить
                </button>
              </div>
            </div>
          </div>
        ) : null}

        <nav className="top-tabs glass-panel" aria-label="Основные вкладки">
          <div className="top-tabs__icon-shell">
            <img
              className="top-tabs__icon"
              src={dashboardIcon}
              alt=""
              aria-hidden="true"
            />
          </div>

          {tabs.map((tab) => {
            const isActive = activeTab === tab.id;

            return (
              <button
                key={tab.id}
                type="button"
                className={`top-tabs__button ${requestClass(tab.requestId)} ${isActive ? "is-active" : ""}`.trim()}
                data-request-id={tab.requestId}
                aria-current={isActive ? "page" : undefined}
                onClick={() => {
                  handleTabClick(tab.id, tab.requestId);
                }}
              >
                {tab.label}
              </button>
            );
          })}
        </nav>

        <div className="main-layout">
          <div className="left-column">
            <div className="clock-card glass-panel" aria-label="Таймер миссии">
              <time dateTime="PT0H0M0S">00:00:00</time>
            </div>

            {activeTab === "main" ? (
              <div
                className={`start-panel__card glass-panel ${requestState === "pending" ? "is-pending" : ""}`.trim()}
              >
                <p className="start-panel__label">Запуск автономного режима</p>

                <button
                  type="button"
                  className={`start-panel__button ${requestClass(requestIds.startAutonomous)}`.trim()}
                  data-request-id={requestIds.startAutonomous}
                  onClick={() => {
                    handleStaticRequestClick(requestIds.startAutonomous);
                  }}
                  aria-label="Запуск"
                >
                  <img
                    className="start-panel__image"
                    src={startButtonImage}
                    alt=""
                    aria-hidden="true"
                  />
                </button>

                {lastSlaveResponse ? (
                  <p className="start-panel__response" aria-live="polite">
                    Ответ slave: {lastSlaveResponse}
                  </p>
                ) : null}
              </div>
            ) : null}

            <div className="battery-widget">
              <div
                className="battery-widget__card glass-panel"
                aria-label={`Заряд робота ${clampPercent(batteryPercent)} процентов, ${(
                  systemSummary.voltage_cV / 100
                ).toFixed(2)} вольт`}
              >
                <svg
                  className="battery-widget__ring"
                  viewBox="0 0 120 120"
                  aria-hidden="true"
                >
                  <circle
                    className="battery-widget__track"
                    cx="60"
                    cy="60"
                    r={batteryRadius}
                    style={{ stroke: batteryTrackColor }}
                  />
                  <circle
                    className="battery-widget__progress"
                    cx="60"
                    cy="60"
                    r={batteryRadius}
                    pathLength="100"
                    style={{
                      stroke: batteryStrokeColor,
                      strokeDashoffset: 100 - clampPercent(batteryPercent),
                    }}
                  />
                </svg>
                <img
                  className="battery-widget__icon"
                  src={batteryIcon}
                  alt=""
                  aria-hidden="true"
                />
              </div>
            </div>
          </div>

          <div className="status-panel glass-panel">
            {renderStatusContent()}
          </div>
        </div>
      </main>
    </div>
  );
}

export default App;
