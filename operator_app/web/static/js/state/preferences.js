const PREFIX = "operator:";

const keyFor = (name) => `${PREFIX}${String(name || "")}`;

export const preferences = {
  getString(name, fallback = "") {
    const value = window.localStorage.getItem(keyFor(name));
    return value === null ? fallback : value;
  },

  getBoolean(name, fallback = false) {
    const value = window.localStorage.getItem(keyFor(name));
    if (value === null) {
      return Boolean(fallback);
    }
    return value !== "0" && value !== "false";
  },

  setString(name, value) {
    window.localStorage.setItem(keyFor(name), String(value ?? ""));
  },

  setBoolean(name, value) {
    window.localStorage.setItem(keyFor(name), value ? "1" : "0");
  },

  remove(name) {
    window.localStorage.removeItem(keyFor(name));
  },
};
