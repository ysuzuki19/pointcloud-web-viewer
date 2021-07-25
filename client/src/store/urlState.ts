import { atom } from 'recoil';

export type urlType = {
  address: string;
  port: string;
};

export const urlState = atom({
  key: 'urlState',
  default: {
    address: 'localhost',
    port: '8080',
  },
});
