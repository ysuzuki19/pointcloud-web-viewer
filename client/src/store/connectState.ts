import { atom } from 'recoil';

export type connectType = boolean;

export const connectState = atom({
  key: 'connectState',
  default: false,
});
