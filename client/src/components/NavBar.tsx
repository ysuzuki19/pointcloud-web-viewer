import React from 'react';
import { useRecoilState } from 'recoil';
import {
  alpha,
  makeStyles,
  Theme,
  createStyles,
} from '@material-ui/core/styles';
import AppBar from '@material-ui/core/AppBar';
import Toolbar from '@material-ui/core/Toolbar';
import Box from '@material-ui/core/Box';
import IconButton from '@material-ui/core/IconButton';
import Typography from '@material-ui/core/Typography';
import InputBase from '@material-ui/core/InputBase';
import MenuIcon from '@material-ui/icons/Menu';

import { urlState, urlType } from '../store/urlState';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    grow: {
      flexGrow: 1,
    },
    menuButton: {
      marginRight: theme.spacing(2),
    },
    title: {
      display: 'none',
      [theme.breakpoints.up('sm')]: {
        display: 'block',
      },
    },
    search: {
      position: 'relative',
      borderRadius: theme.shape.borderRadius,
      backgroundColor: alpha(theme.palette.common.white, 0.15),
      '&:hover': {
        backgroundColor: alpha(theme.palette.common.white, 0.25),
      },
      marginRight: theme.spacing(0),
      marginLeft: 0,
      width: '100%',
      [theme.breakpoints.up('sm')]: {
        marginLeft: theme.spacing(0.5),
        width: 'auto',
      },
    },
    inputRoot: {
      color: 'inherit',
    },
    inputInput: {
      padding: theme.spacing(0, 0, 0, 0),
      // vertical padding + font size from searchIcon
      paddingLeft: `calc(0em + ${theme.spacing(1)}px)`,
      transition: theme.transitions.create('width'),
      width: '100%',
      [theme.breakpoints.up('md')]: {
        width: '12ch',
      },
    },
    sectionDesktop: {
      display: 'none',
      [theme.breakpoints.up('md')]: {
        display: 'flex',
      },
    },
    sectionMobile: {
      display: 'flex',
      [theme.breakpoints.up('md')]: {
        display: 'none',
      },
    },
  })
);

const NavBar: React.FC = () => {
  const classes = useStyles();
  const [url, setUrl] = useRecoilState<urlType>(urlState);

  const updateAddress = (e: any) => {
    setUrl({ address: e.target.value, port: url.port });
  };

  const updatePort = (e: any) => {
    setUrl({ address: url.address, port: e.target.value });
  };

  return (
    <div className={classes.grow}>
      <AppBar position="static">
        <Toolbar>
          <IconButton
            edge="start"
            className={classes.menuButton}
            color="inherit"
            aria-label="open drawer"
          >
            <MenuIcon />
          </IconButton>
          <Typography className={classes.title} variant="h6" noWrap>
            Point Cloud Web Viewer
          </Typography>
          <Box pl={5}>
            <Typography className={classes.title} variant="h6" noWrap>
              ws://
            </Typography>
          </Box>
          <div className={classes.search}>
            <InputBase
              placeholder="address"
              classes={{
                root: classes.inputRoot,
                input: classes.inputInput,
              }}
              inputProps={{ 'aria-label': 'search' }}
              value={url.address}
              onChange={updateAddress}
            />
          </div>
          <Typography className={classes.title} variant="h6" noWrap>
            :
          </Typography>
          <div className={classes.search}>
            <InputBase
              placeholder="port"
              classes={{
                root: classes.inputRoot,
                input: classes.inputInput,
              }}
              inputProps={{ 'aria-label': 'search' }}
              value={url.port}
              onChange={updatePort}
            />
          </div>
          <div className={classes.grow} />
        </Toolbar>
      </AppBar>
    </div>
  );
};

export default NavBar;
