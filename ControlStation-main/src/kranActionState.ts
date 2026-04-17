export type KranActionTimeoutContext = {
  requestId: string;
  statusCode?: number;
  errorMessage?: string;
};

const kranActionRequestIdPattern = /^(11|12)_/;

const isKranActionRequestId = (requestId: string) =>
  kranActionRequestIdPattern.test(requestId);

const isTimeoutErrorMessage = (errorMessage?: string) =>
  errorMessage?.trim().toLowerCase().includes("timeout") ?? false;

export const shouldResetKranActionToHomeOnSuccess = (requestId: string) =>
  isKranActionRequestId(requestId);

export const shouldResetKranActionToHomeOnFailure = (requestId: string) =>
  isKranActionRequestId(requestId);

export const shouldResetKranActionToHomeOnTimeout = ({
  requestId,
  statusCode,
  errorMessage,
}: KranActionTimeoutContext) =>
  isKranActionRequestId(requestId) &&
  (statusCode === 504 || isTimeoutErrorMessage(errorMessage));
