export class CommandStack {
  constructor(limit = 100) {
    this.limit = Math.max(1, Number(limit) || 100);
    this.undoCommands = [];
    this.redoCommands = [];
    this.onChange = null;
  }

  get canUndo() {
    return this.undoCommands.length > 0;
  }

  get canRedo() {
    return this.redoCommands.length > 0;
  }

  clear() {
    this.undoCommands = [];
    this.redoCommands = [];
    this.notify();
  }

  push(command) {
    if (!command || typeof command.undo !== "function" || typeof command.redo !== "function") {
      return false;
    }
    this.undoCommands.push(command);
    if (this.undoCommands.length > this.limit) {
      this.undoCommands.shift();
    }
    this.redoCommands = [];
    this.notify();
    return true;
  }

  undo() {
    const command = this.undoCommands[this.undoCommands.length - 1];
    if (!command) {
      return false;
    }
    // Apply first so a failed command remains available to retry instead of
    // silently disappearing from the history.
    command.undo();
    this.undoCommands.pop();
    this.redoCommands.push(command);
    this.notify();
    return true;
  }

  redo() {
    const command = this.redoCommands[this.redoCommands.length - 1];
    if (!command) {
      return false;
    }
    // Keep both stacks unchanged when the command cannot be restored.
    command.redo();
    this.redoCommands.pop();
    this.undoCommands.push(command);
    this.notify();
    return true;
  }

  notify() {
    if (typeof this.onChange === "function") {
      this.onChange(this);
    }
  }
}
