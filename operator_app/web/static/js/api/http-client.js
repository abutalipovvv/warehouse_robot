export class HttpError extends Error {
  constructor(message, options = {}) {
    super(message);
    this.name = "HttpError";
    this.status = Number(options.status || 0);
    this.url = String(options.url || "");
    this.payload = options.payload ?? null;
  }
}

export class HttpClient {
  constructor(options = {}) {
    this.defaultHeaders = {
      Accept: "application/json",
      ...(options.defaultHeaders || {}),
    };
    this.cache = options.cache || "no-store";
  }

  get(url, options = {}) {
    return this.request(url, { ...options, method: "GET" });
  }

  post(url, payload, options = {}) {
    return this.request(url, {
      ...options,
      method: "POST",
      body: payload,
    });
  }

  delete(url, options = {}) {
    return this.request(url, { ...options, method: "DELETE" });
  }

  async request(url, options = {}) {
    const request = {
      method: options.method || "GET",
      cache: options.cache || this.cache,
      headers: {
        ...this.defaultHeaders,
        ...(options.headers || {}),
      },
    };
    if (options.body !== undefined) {
      request.headers["Content-Type"] = request.headers["Content-Type"] || "application/json";
      request.body = request.headers["Content-Type"].includes("application/json")
        ? JSON.stringify(options.body)
        : options.body;
    }
    const response = await fetch(url, request);
    const payload = await response.json().catch(() => ({}));
    const rejectApplicationError = options.rejectApplicationError !== false;
    if (!response.ok || (rejectApplicationError && payload?.ok === false)) {
      throw new HttpError(payload?.error || `Request failed: ${response.status}`, {
        status: response.status,
        url,
        payload,
      });
    }
    return payload;
  }
}

export const httpClient = new HttpClient();
