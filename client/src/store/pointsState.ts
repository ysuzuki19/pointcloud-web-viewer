import { atom } from 'recoil';

export type pointType = [number, number, number];
export type pointsType = Array<pointType>;

export const pointsState = atom({
  key: 'pointsState',
  default: [] as pointsType,
});
