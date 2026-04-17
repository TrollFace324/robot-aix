export type TelemetryBoomSlotOption = {
  slot: number;
  label: string;
};

export const telemetryBoomSlotCount = 11;
export const telemetryBoomMinMm = 0;
export const telemetryBoomMaxMm = 2000;

export const formatTelemetryBoomSlotLabel = (slot: number) =>
  `Позиция ${slot}`;

export const buildTelemetryBoomSlotOptions = (): TelemetryBoomSlotOption[] =>
  Array.from({ length: telemetryBoomSlotCount }, (_, slot) => ({
    slot,
    label: formatTelemetryBoomSlotLabel(slot),
  }));

export const parseTelemetryBoomSlots = (value: unknown): number[] | null => {
  if (!Array.isArray(value) || value.length !== telemetryBoomSlotCount) {
    return null;
  }

  const parsed = value.map((entry) => {
    if (typeof entry !== "number" || !Number.isFinite(entry)) {
      return null;
    }

    return Math.max(0, Math.round(entry));
  });

  return parsed.every((entry): entry is number => typeof entry === "number")
    ? parsed
    : null;
};

type TelemetryBoomSyncState = {
  isTelemetryTabActive: boolean;
  isBoomEditorSelected: boolean;
  isSystemSummaryStale: boolean;
};

export const shouldLoadTelemetryBoomSlots = ({
  isTelemetryTabActive,
  isBoomEditorSelected,
  isSystemSummaryStale,
}: TelemetryBoomSyncState) =>
  isTelemetryTabActive && isBoomEditorSelected && !isSystemSummaryStale;

export const shouldClearTelemetryBoomSlots = ({
  isTelemetryTabActive,
  isBoomEditorSelected,
  isSystemSummaryStale,
}: TelemetryBoomSyncState) =>
  isTelemetryTabActive && isBoomEditorSelected && isSystemSummaryStale;

const sanitizeTelemetryMmDraft = (value: string) =>
  value.replace(/\D/g, "").slice(0, 4);

export const isTelemetryBoomMmInRange = (value: number) =>
  Number.isInteger(value) &&
  value >= telemetryBoomMinMm &&
  value <= telemetryBoomMaxMm;

export const normalizeTelemetryMmDraft = (value: string) => {
  const digits = sanitizeTelemetryMmDraft(value);
  if (digits.length === 0) {
    return "0";
  }

  return String(Number(digits));
};

export const appendDigitToTelemetryMmDraft = (
  currentValue: string,
  digit: string,
) => {
  if (!/^\d$/.test(digit)) {
    return normalizeTelemetryMmDraft(currentValue);
  }

  const normalizedCurrentValue = normalizeTelemetryMmDraft(currentValue);
  const nextValue =
    normalizedCurrentValue === "0"
      ? digit
      : `${normalizedCurrentValue}${digit}`;

  return sanitizeTelemetryMmDraft(nextValue);
};

export const backspaceTelemetryMmDraft = (currentValue: string) => {
  const digits = sanitizeTelemetryMmDraft(currentValue);
  if (digits.length <= 1) {
    return "0";
  }

  return digits.slice(0, -1);
};
